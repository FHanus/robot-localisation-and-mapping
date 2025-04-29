% Filename: ekf_localisation.m
% Author: Filip Hanuš
% Description: Task 4 script implementing Extended Kalman Filter-based localisation; loads Task 3 dataset, runs EKF prediction and update loops, and plots pose trajectories and error comparisons.

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

% Initial covariance 
Sigma = diag([0.001, 0.001, 0.001]); 

% Standard deviations   
sigma_r   = 0.2;              
sigma_phi = 5 * pi/180;      

% Measurement noise def
Qt = diag([ sigma_r^2, sigma_phi^2 ]);  

% Init
state_size = 3; % [x; y; theta]
num_steps = size(robot_pose_history, 2);

% Initialise state and covariance
mu = robot_pose_history(:, 1); % Initial state estimate

% Store results
mu_history = zeros(state_size, num_steps);
mu_history(:, 1) = mu;
Sigma_history = zeros(state_size, state_size, num_steps);
Sigma_history(:, :, 1) = Sigma;

% Main EKF Loop
for t = 2:num_steps
    % get previous velocity command
    u = vel_sent_history(:, t-1); % [v; omega]
    v = u(1);
    omega = u(2);
    
    % Time step
    dt = time_history(t-1); 

    %% EKF prediction step
    % Perfectly following formula from practical 17 / PR Ch 7.5, Eq 7.3
    theta_prev = mu(3);

    % Calculate Motion Jacobian Gt
    if abs(omega) < 1e-6 % Jacobian for straight motion
         G = [1 0 -v * dt * sin(theta_prev);
              0 1  v * dt * cos(theta_prev);
              0 0  1];
    else
        radius = v/omega;
         G = [1 0 -radius * cos(theta_prev) + radius * cos(theta_prev + omega*dt);
              0 1 -radius * sin(theta_prev) + radius * sin(theta_prev + omega*dt);
              0 0  1];
    end

    % Calculate Jacobian Vt
    if abs(omega) < 1e-6 % Jacobian for straight motion
        V = [dt*cos(theta_prev) 0;
             dt*sin(theta_prev) 0;
             0                  0]; 
    else
        V = [(-sin(theta_prev) + sin(theta_prev + omega*dt))/omega,  (v*(sin(theta_prev) - sin(theta_prev + omega*dt)))/omega^2 + (v*dt*cos(theta_prev + omega*dt))/omega;
             (cos(theta_prev) - cos(theta_prev + omega*dt))/omega, -(v*(cos(theta_prev) - cos(theta_prev + omega*dt)))/omega^2 + (v*dt*sin(theta_prev + omega*dt))/omega;
             0,                                                   dt];
    end

    % Calculate motion noise covariance
    M = diag([alphas(1)*v^2 + alphas(2)*omega^2, alphas(3)*v^2 + alphas(4)*omega^2]);
       
    % Predicted pose, repeated velocity model
    % Commented better in sample_velocity_motion_model.m in task2
    if abs(omega) < 1e-6 % Avoid division by zero for straight motion -> 1e-6 
        dx_pred = v * dt * cos(theta_prev);
        dy_pred = v * dt * sin(theta_prev);
        dtheta_pred = 0;
    else
        radius = v / omega;
        dx_pred = -radius * sin(theta_prev) + radius * sin(theta_prev + omega * dt);
        dy_pred =  radius * cos(theta_prev) - radius * cos(theta_prev + omega * dt);
        dtheta_pred = omega * dt;
    end
    mu_bar = mu + [dx_pred; dy_pred; dtheta_pred];
    
    % Predict covariance Sigma_bar 
    Sigma_bar = G * Sigma * G' + V * M * V';

    %% EKF Update Step
    % Still following formulas from practical 17 / PR Ch 7.5, Eq 7.3

    % Find marker IDs that are not zero at this step
    current_measurement_data = aruco_history(:, :, t);
    valid_marker_indices = find(current_measurement_data(1, :) > 0);

    % For each of the observed features 
    for idx = valid_marker_indices
        % (in PR it loops through ranges, bearings and IDs, we do that below)
        marker_id = current_measurement_data(1, idx);

        % Convert loaded X/Y -> measurement zt 
        marker_x_approx_base = current_measurement_data(2, idx); 
        marker_y_approx_base = current_measurement_data(3, idx); 
        % Pythagoras
        zt_range = sqrt(marker_x_approx_base^2 + marker_y_approx_base^2);
        zt_bearing = atan2(marker_y_approx_base, marker_x_approx_base); 
        zt = [zt_range; zt_bearing]; % Measurement [range; bearing]
        
        % Find marker position [mx, my] from generated map
        marker_pos = aruco_map(marker_id, 2:3)'; % [mx; my]

        % Distance q between predicted and true 
        dx_m = marker_pos(1) - mu_bar(1); 
        dy_m = marker_pos(2) - mu_bar(2);
        q = dx_m^2 + dy_m^2;

        % Calculate predicted measurement z_hat (to simplify, we dont need the ID m_js which is in the book) 
        predicted_range = sqrt(q);
        predicted_bearing = atan2(dy_m, dx_m) - mu_bar(3);
        z_hat = [predicted_range; predicted_bearing];

        % Calculate Measurement Jacobian Ht
        H = [-dx_m/predicted_range, -dy_m/predicted_range, 0;
              dy_m/q              , -dx_m/q             , -1];
        
        % Calculate Kalman Gain Kt
        K = Sigma_bar * H' / (H * Sigma_bar * H' + Qt); 

        % Innovation (handle angle wrapping otherwise there are larger errors)
        innovation = zt - z_hat;
        innovation(2) = wrapToPi(innovation(2)); 

        % Update state estimate mu
        mu_bar = mu_bar + K * innovation;

        % Update covariance Sigma
        Sigma_bar = (eye(state_size) - K * H) * Sigma_bar;

    end 
    
    % Final state and covariance for this time step
    mu = mu_bar;
    Sigma = Sigma_bar; 
    
    % Store results
    mu_history(:, t) = mu;
    Sigma_history(:, :, t) = Sigma;
    
    % Skipped p_zt
end

% Plot results
figure(1);
clf;
hold on;
plot(robot_pose_history(1,:), robot_pose_history(2,:), 'b-', 'LineWidth', 1.5);
plot(noisy_pose_history(1,:), noisy_pose_history(2,:), 'r:', 'LineWidth', 1);
plot(mu_history(1,:), mu_history(2,:), 'g-', 'LineWidth', 1.5);
hold off;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Task 4: Ground Truth, Noisy Estimate, and EKF Estimate');
legend('Ground Truth', 'Noisy Motion Model', 'EKF Estimate', 'Location', 'northwest');
axis equal;
grid on;

% Plot L2 Norm Error Comparison
% Extract Xs,Ys
gt = robot_pose_history(1:2, :); 
noisy = noisy_pose_history(1:2, :);
ekf = mu_history(1:2, :);
% L2 norm along columns
error_noisy = vecnorm(gt - noisy, 2, 1); 
error_ekf = vecnorm(gt - ekf, 2, 1);

figure(2);
clf;
hold on;
plot(1:num_steps, error_noisy, 'r-', 'LineWidth', 1);
plot(1:num_steps, error_ekf, 'g-', 'LineWidth', 1.5);
hold off;
xlabel('Time Step');
ylabel('L2 Norm Position Error (m)');
title('Task 4: L2 Norm Error Comparison: Noisy vs EKF Estimate');
legend('L2NORM(Ground Truth - Noisy Estimate)', 'L2NORM(Ground Truth - EKF Estimate)', 'Location', 'best');
grid on;