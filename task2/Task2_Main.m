% Filename: Task2_Main.m
% Author: Filip Hanuš
% Description: Main script for Task 2: applies the velocity-motion model to Task 1 data, saves noisy pose history, and plots ground truth vs noisy paths.

clear;
clc;
close all;

% Path to the Task 1 data file
task1_data_filename = 'task1/secondary_task1_data.mat'; 
load(task1_data_filename, 'robot_pose_history', 'vel_sent_history', 'time_history');

% Generate Noisy Path

% Define the noise parameters
% [ α1 = 0.1, α2 = 0.02, α3 = 0.3, α4 = 0.01, α5 = 0.001, α6 = 0.01 ]
alphas = [0.1, 0.02, 0.3, 0.01, 0.001, 0.01]; 

% Get total time steps from data
num_steps = size(robot_pose_history, 2);

% Prep noisy pose history var
noisy_pose_history = zeros(3, num_steps);

% Start the noisy path at the same initial pose as the ground truth
noisy_pose_history(:, 1) = robot_pose_history(:, 1); 

% Loop through each time step
loop_rate_hz = 30; 
dt = 1 / loop_rate_hz; 

for t = 2:num_steps % Start from second step 
    % Get previous (noisy) pose
    prev_noisy_pose = noisy_pose_history(:, t-1);
    
    % Get the previous velocity command 
    velocity_command = vel_sent_history(:, t-1);
    
    % Predict next noisy pose 
    current_noisy_pose = sample_velocity_motion_model(prev_noisy_pose, velocity_command, alphas, dt);
    
    % Store the calculated noisy pose for the current step
    noisy_pose_history(:, t) = current_noisy_pose;
end

% Save noisy path data
save('task2/task2_data.mat', 'noisy_pose_history', 'robot_pose_history', 'alphas', 'dt');

% Plot ground truth vs noisy path 

figure(2);
clf; 
hold on; 

% Plot the ground truth
plot(robot_pose_history(1,:), robot_pose_history(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth Path (Task 1)'); 

% Plot the noisy path
plot(noisy_pose_history(1,:), noisy_pose_history(2,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Noisy Path (Velocity Model)'); 

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Task 2: Ground Truth vs. Noisy Velocity Model Path');

legend('Location', 'best');

axis equal;
grid on;
hold off;