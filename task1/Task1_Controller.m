% Filename: Task1_Controller.m
% Author: Filip Hanuš
% Description: Closed-loop ROS/MATLAB robot controller for Task 1 (exploration and collision avoidance).
% Logs ground truth poses, sent velocity commands, and execution timing; plots the robot path.

global GL_robot_pose;   % Global odometry callback data - [x, y, theta] 
global GL_ranges;       % Global lidar callback data - [front_min, left_min, right_min] 

% Init defaults
GL_robot_pose = [0 0 0];
GL_ranges = [inf inf inf];

% ROS publisher for sending velocity commands
% Messages - type geometry_msgs/Twist that go to the topic /cmd_vel 
[vel_pub, vel_msg] = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

% ROS subscriber for lidar scans
% Messages - type sensor_msgs/LaserScan from topic /scan  
% Calls ROSRangeCallback function when a message arrives
range_sub = rossubscriber('/scan', 'sensor_msgs/LaserScan', @ROSRangeCallback, "DataFormat", "struct");

% ROS subscriber for odometry data
% Messages - type nav_msgs/Odometry from topic /odom
% Calls ROSPose_GazeboCallback function when a message arrives
odom_sub = rossubscriber('/odom', 'nav_msgs/Odometry', @ROSPose_GazeboCallback, "DataFormat", "struct");

% Reset (for rerunning)
resetclient = rossvcclient('/gazebo/reset_world');
resetmsg = rosmessage(resetclient); % Setup rst message
call(resetclient, resetmsg, 'Timeout', 5); % Call service with timeout

% Controller settings
n_iterations = 6500;         % Number of control loop iterations (tried to time so it finishes the maze)
loop_rate_hz = 30;           % Control frequency (Hz)
pause_duration = 1/loop_rate_hz; 
sensor_max_range = 3.5;      % Max lidar range that replaces broken data

% Control parameters
collision_threshold = 0.44;  % Front detection distance
go_straight_threshold = 1.5; % Front "open" distance
Kp_center_turn = 0.3;        % Centring speed gain (tuned to keep at center smoothly)
Kp_linear = 0.2;             % Forward speed gain 
max_linear_velocity = 0.25;  % Max forward speed
max_center_angular_velocity = 0.6; % Max angular speed
obstacle_turn_speed = 0.4;   % Fixed speed for turns

% Data logging setup
robot_pose_history = zeros(3, n_iterations); % For [x; y; theta] 
vel_sent_history   = zeros(2, n_iterations); % For [linear.X; angular.Z]
time_history       = zeros(1, n_iterations); % Time taken per loop

% Main control loop
disp("Starting control loop...");
for i = 1:n_iterations
    tic; % Start timer
    
    % Read sensors
    front_distance = GL_ranges(1);
    left_distance  = GL_ranges(2); 
    right_distance = GL_ranges(3);
    
    % Handle infinite values
    if isinf(front_distance), front_distance = sensor_max_range; end
    if isinf(left_distance), left_distance = sensor_max_range; end
    if isinf(right_distance), right_distance = sensor_max_range; end 
    
    % Control logic
    if front_distance < collision_threshold
        % Priority 1 - Wall infront
        linear_velocity = 0.0; % Stop
        
        % Turn towards the side with more space
        if left_distance > right_distance + 0.1 % +0.1 to prevent oscillations
            angular_velocity = obstacle_turn_speed;
            disp('Wall infront - turning left');
        else
            angular_velocity = -obstacle_turn_speed;
            disp('Wall infront - turning right');
        end

    elseif front_distance > go_straight_threshold
        % Priority 2 - Open path infront
        angular_velocity = 0.0;

        % Velocity based on front distance
        linear_velocity = min(front_distance * Kp_linear, max_linear_velocity);

        % Handle negative hopefully not needed
        %linear_velocity = max(linear_velocity, 0.0); 
        disp('Path clear - going forward');
        
    else
        % Priority 3 - Keep centred
        % Angular velocity based on distance between walls
        centering_error = left_distance - right_distance;
        angular_velocity = Kp_center_turn * centering_error;
        
        % Velocity based on front distance
        linear_velocity = min(front_distance * Kp_linear, max_linear_velocity);
        %linear_velocity = max(linear_velocity, 0.0); % Cap negative, hopefully not needed
        
        % Limit angular velocity for centering
        % Cap too high
        angular_velocity = min(angular_velocity, max_center_angular_velocity); 
        % Cap too low
        angular_velocity = max(angular_velocity, -max_center_angular_velocity); 
        
        disp('Path clear - centering');
    end

    % Assign velocities to the message structure
    vel_msg.Linear.X = linear_velocity;
    vel_msg.Angular.Z = angular_velocity;

    % Send command
    send(vel_pub, vel_msg);
    
    % Log the current ground truth and sent cmds
    robot_pose_history(:, i) = GL_robot_pose'; % Store current (' for column)
    vel_sent_history(:, i)   = [vel_msg.Linear.X; vel_msg.Angular.Z]; % Store velocity
    time_history(:, i)       = toc; % Store time since 'tic' (loop time)
    
    % Pause
    time_elapsed = toc;
    pause_time = max(0, pause_duration - time_elapsed); % with 0 handling
    pause(pause_time); 

    time_history(:, i) = time_elapsed + pause_time; % Store total time with pause
end
disp("Control loop finished.");

% Stop
vel_msg.Linear.X = 0;   
vel_msg.Angular.Z = 0;   
send(vel_pub, vel_msg);

% Save logged
save('secondary_task1_data.mat', 'robot_pose_history', 'vel_sent_history', 'time_history');

% Plotting
figure(1);                          
clf;                                
plot(robot_pose_history(1,:), robot_pose_history(2,:), 'r-'); 
xlabel('X Position (m)');           
ylabel('Y Position (m)');           
title('Task 1: Robot Ground Truth Path'); 
axis equal;                         
grid on;                            
legend('Robot Path');               

% Shutdown ROS
rosshutdown;
disp("ROS Shut Down.");