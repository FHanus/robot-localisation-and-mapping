% Filename: pf_localisation.m
% Author: Filip Hanuš
% Description: Task 5 script implementing Particle Filter-based localisation; loads Task 3 dataset, runs PF prediction, weighting, resampling, and plots pose trajectories and error comparisons.

clear all;
close all;
clc; 

% Directory with dataset  from task 3
data_dir = 'task3_dataset'; 

% Load data from individual files using fullfile
load(fullfile(data_dir,'ground_truth.mat')); % Loads robot_pose_history
load(fullfile(data_dir,'noisy_pose.mat'));   % Loads noisy_pose_history
load(fullfile(data_dir,'commands.mat'));     % Loads vel_sent_history
load(fullfile(data_dir,'aruco.mat'));         % Loads aruco_history
load(fullfile(data_dir,'aruco_map.mat'),'aruco_map');     % Loads aruco_map - landmark map [ID X Y]
load(fullfile(data_dir,'time.mat'));           % Loads time_history

% Define the noise parameters
% [ α1 = 0.1, α2 = 0.02, α3 = 0.3, α4 = 0.01, α5 = 0.001, α6 = 0.01 ]
alphas = [0.1, 0.02, 0.3, 0.01, 0.001, 0.01]; 

% Initial uncertainty 
initial_uncertainty_stddev = [0.001; 0.001; 0.001];

% Standard deviations 
range_std_dev = 0.2;         
bearing_std_dev = 5 * pi/180; 

% Particle filter parameters 
num_particles = 100000; % 10 bad, 100 good, 1000 good enough

% Store results
num_steps = size(robot_pose_history, 2); 
pf_pose_history = zeros(3, num_steps); % [x; y; theta]

% Set the initial belief center to the first ground truth pose
initial_pose = robot_pose_history(:, 1); 

% Generate the initial particle set -> sample around initial_pose
particles = [initial_pose(1:2) + initial_uncertainty_stddev(1:2) .* randn(2, num_particles); % x and y
             initial_pose(3) + initial_uncertainty_stddev(3) * randn(1, num_particles)]; % theta

% Initialise particle weights 
weights = ones(num_particles, 1) / num_particles; % Mx1 sum(weights)=1

% Store the initial pose estimate
pf_pose_history(:, 1) = particles * weights; 

% Main particle filter loop Probabilistic Robotics, table 4.3
for t = 1:(num_steps - 1) % Loop through time steps t
    %% Prediction Step 
    % Apply the motion model to predict the next pose for each particle
    u_t = vel_sent_history(:, t); % Get control command 
    dt = time_history(t);          % Get time step 

    % Apply noisy motion model to each particle independently.
    for i = 1:num_particles
        % Velocity motion model sampling algorithm, Table 5.3; Page 124 Thrun's Probabilistic Robotics
        % Commented better in sample_velocity_motion_model.m in task2
        % Calculate std dev for noise added to linear velocity v
        v_std_dev_sq = (alphas(1)*u_t(1)^2 + alphas(2)*u_t(2)^2);
        % Calculate std dev for noise added to angular velocity w
        w_std_dev_sq = (alphas(3)*u_t(1)^2 + alphas(4)*u_t(2)^2);
        % Calculate std dev for the final rotational noise gamma
        gamma_std_dev_sq = (alphas(5)*u_t(1)^2 + alphas(6)*u_t(2)^2);
    
        % Sample noisy velocities from Gaussian distributions
        v_hat = u_t(1) + sqrt(v_std_dev_sq) * randn(); 
        w_hat = u_t(2) + sqrt(w_std_dev_sq) * randn();
        gamma_hat = sqrt(gamma_std_dev_sq) * randn();
    
        % Predict next pose
        x_prev = particles(1, i);      
        y_prev = particles(2, i);      
        theta_prev = particles(3, i); 
     
        if abs(w_hat) < 1e-6 % Straight-line motion
            x_next = x_prev + v_hat * dt * cos(theta_prev);
            y_next = y_prev + v_hat * dt * sin(theta_prev);
            theta_next_no_gamma = theta_prev;
        else
            radius = v_hat / w_hat;
            x_next = x_prev - radius * sin(theta_prev) + radius * sin(theta_prev + w_hat * dt);
            y_next = y_prev + radius * cos(theta_prev) - radius * cos(theta_prev + w_hat * dt);
            theta_next_no_gamma = theta_prev + w_hat * dt;
        end
        theta_next = theta_next_no_gamma + gamma_hat * dt;
        particles(:, i) = [x_next; y_next; theta_next];
    end

    %% Update Step 
    % Weighting step (line 5 of the table 4.3)
    % Get aruco measurement at time t+1
    markers_at_t_plus_1 = aruco_history(:, :, t+1); 
    
    % Find detections (ID > 0)
    valid_detections_indices = find(markers_at_t_plus_1(1, :) > 0);
        
    % Array to store the weight factor for each particle
    particle_unnormalized_weights = ones(num_particles, 1); 

    % If there are valid measurements at time t+1 update weights
    if any(valid_detections_indices)
        % Calculate the importance weight factor for each particle
        for i = 1:num_particles 
            particle_pose = particles(:, i); % Current particle pose
            
            % Total weight factor for this particle
            total_particle_weight_factor = 1.0; 

            % Iterate through each detected ArUco marker
            for idx_in_slice = valid_detections_indices 
                marker_id = markers_at_t_plus_1(1, idx_in_slice); 
                
                % 1. Get the actual measurement [range; bearing] just like before in ekf
                marker_x_relative = markers_at_t_plus_1(2, idx_in_slice); 
                marker_y_relative = markers_at_t_plus_1(3, idx_in_slice); 
                zt_range   = sqrt(marker_x_relative^2 + marker_y_relative^2);
                zt_bearing = atan2(marker_y_relative, marker_x_relative);
                zt_j = [zt_range; zt_bearing]; 

                % 2. Get the corresponding landmark map position
                map_entry_idx = find(aruco_map(:, 1) == marker_id);
                landmark_pos_world = aruco_map(map_entry_idx, 2:3)'; % [mjx, mjy]
                
                % 3. Calculate expected measurement (range_hat, bearing_hat) 
                %    based on particle pose x[m]_t and landmark position mj
                dx = landmark_pos_world(1) - particle_pose(1); % mjx - x
                dy = landmark_pos_world(2) - particle_pose(2); % mjy - y
                range_hat = sqrt(dx^2 + dy^2);  
                bearing_hat = atan2(dy, dx) - particle_pose(3);
                
                % 4. Calculate measurement error
                range_error   = zt_j(1) - range_hat;   
                bearing_error = zt_j(2) - bearing_hat;
                
                % 5. Calculate the probability of this measurement if the robot was at the particle pose
                range_var   = range_std_dev^2;   % Expected variance in range 
                bearing_var = bearing_std_dev^2; % Expected variance in bearing

                % Calculate range match score
                % 1 / (1 + (squared_error / variance))
                range_match_factor = 1 / (1 + (range_error^2) / range_var);
                % Calculate bearing match score
                bearing_match_factor = 1 / (1 + (bearing_error^2) / bearing_var);

                p = (range_match_factor * bearing_match_factor); 
                                
                % 6. Accumulate probability p(zt|x)
                total_particle_weight_factor = total_particle_weight_factor * p;            
            end
            % Store the calculated total weight for particle (Table 4.3 line 5)
            particle_unnormalized_weights(i) = total_particle_weight_factor;
        end 
        % Normalise the collected weights so they sum to 1
        sum_weights = sum(particle_unnormalized_weights);
        if sum_weights > eps % Avoid division by zero
            weights = particle_unnormalized_weights / sum_weights;
        end
    end 

    %% Resampling Step
    % Implements systematic resampling (algorithm particle filter, lines 8-11), and low variance resampling table 4.4
  
    % Normalise weights
    sum_weights_resample = sum(weights);
    weights = weights / sum_weights_resample; 

    % Calculate cumulative distribution function 
    % Cumulative sums for c in Lines 4 & 10 of table 4.4
    cdf = cumsum(weights);
    
    % Generate sampling pointers ->> the core line 6 of table 4.4
    start_point = rand / num_particles; % Random start
    pointers = start_point + (0:(num_particles-1))' / num_particles; % M pointers

    % Select particles based on pointers and CDF
    output_indices = zeros(num_particles, 1);
    cdf_idx = 1; 
    for ptr_idx = 1:num_particles % For each pointer
        % Move along CDF to find the segment with the pointer
        while pointers(ptr_idx) > cdf(cdf_idx) && cdf_idx < num_particles
            cdf_idx = cdf_idx + 1;
        end
        output_indices(ptr_idx) = cdf_idx; % Store index of selected particle
    end
    % Create the new particle set
    new_particles_temp = particles(:, output_indices);
    
    % Reset weights 
    new_weights_temp = ones(num_particles, 1) / num_particles;

    % Update main loop variables
    particles = new_particles_temp;
    weights   = new_weights_temp;

    % Calculate the single best pose estimate from the resampled particle set
    pf_pose_history(:, t+1) = particles * weights;    
end 

% Plot path comparison 
figure(1);
clf; 
hold on; 
plot(robot_pose_history(1,:), robot_pose_history(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth'); 
plot(noisy_pose_history(1,:), noisy_pose_history(2,:), 'r:', 'LineWidth', 1, 'DisplayName', 'Noisy Motion Model'); 
plot(pf_pose_history(1,:), pf_pose_history(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'PF Estimate'); 

title('Task 5: Robot Paths: Ground Truth, Noisy Estimate, and PF Estimate'); 
xlabel('X Position (m)'); 
ylabel('Y Position (m)'); 
legend('show', 'Location', 'northwest'); 
axis equal; 
grid on; 
hold off;

% Calculate and plot L2 norm errors
num_steps_err = size(robot_pose_history, 2);
error_noisy = zeros(1, num_steps_err);
error_pf = zeros(1, num_steps_err);

for t = 1:num_steps_err
    % Error for noisy path vs ground truth
    error_noisy(t) = norm(noisy_pose_history(1:2, t) - robot_pose_history(1:2, t)); 
    error_pf(t) = norm(pf_pose_history(1:2, t) - robot_pose_history(1:2, t)); 
end

% Plot errors over time
figure(2);
clf;
hold on;
time_vector_err = cumsum([0, time_history(1:num_steps_err-1)]); 
plot(time_vector_err, error_noisy, 'r-', 'LineWidth', 1, 'DisplayName', 'L2NORM(Ground Truth - Noisy Estimate)');
plot(time_vector_err, error_pf, 'g-', 'LineWidth', 1.5, 'DisplayName', 'L2NORM(Ground Truth - PF Estimate)');

title('Task 5: L2 Norm Error Comparison: Noisy vs PF Estimate');
xlabel('Time (s)'); 
ylabel('L2 Norm Position Error (m)');
legend('show', 'Location', 'best');
grid on;
hold off;