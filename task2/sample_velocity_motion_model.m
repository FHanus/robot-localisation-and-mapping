% Filename: sample_velocity_motion_model.m
% Author: Filip Hanuš
% Description: Implements the velocity-motion sampling algorithm (Table 5.3, Thrun's Probabilistic Robotics).
%
% Inputs:
%   xt_1: Previous pose [x; y; theta] 
%   ut:   Velocity command [v; omega] -> linear velocity (v) and angular velocity (omega)
%   alphas: Vector of noise parameters [a1, a2, a3, a4, a5, a6] 
%   dt:   Time step duration (s)
%
% Output:
%   xt:   Sampled next pose [x; y; theta]

function xt = sample_velocity_motion_model(xt_1, ut, alphas, dt)
    % Extract from inputs
    x_prev = xt_1(1);     % Previous x-coordinate
    y_prev = xt_1(2);     % Previous y-coordinate
    theta_prev = xt_1(3); % Previous orientation

    % It is (-) because there is a bug that flips all of the velocities (so the result would be mirrored)
    v_cmd = ut(1);       % Commanded linear vel
    omega_cmd = ut(2);    % Commanded angular vel

    a1 = alphas(1); % v_cmd ~> v noise
    a2 = alphas(2); % omega_cmd ~> v noise
    a3 = alphas(3); % v_cmd ~> omega noise
    a4 = alphas(4); % omega_cmd ~> omega noise
    a5 = alphas(5); % v_cmd ~> final theta noise
    a6 = alphas(6); % omega_cmd ~> final theta noise

    % Variance of the noise for the linear velocity
    % Formula from table 5.3 of PR: variance = alpha1 * v_cmd^2 + alpha2 * omega_cmd^2
    v_var = (a1 * v_cmd^2) + (a2 * omega_cmd^2);

    % Sample noisy linear velocity (v_hat)
    % Formula: v_hat = v_cmd + sample(v_var)
    % randn -> sample from N(0, 1)
    % Multiply by sqrt(v_var) (the standard deviation) to scale the noise
    v_hat = v_cmd + (randn * sqrt(v_var));

    % Variance of the noise for the angular velocity
    % Formula: variance = alpha3 * v_cmd^2 + alpha4 * omega_cmd^2
    omega_var = (a3 * v_cmd^2) + (a4 * omega_cmd^2);

    % Sample noisy angular velocity (omega_hat)
    % Formula: omega_hat = omega_cmd + sample(omega_var)
    % omega_cmd + noise sampled from N(0, omega_var)
    omega_hat = omega_cmd + (randn * sqrt(omega_var));

    % Variance of final orientation noise term (gamma)
    % Formula: variance = alpha5 * v_cmd^2 + alpha6 * omega_cmd^2
    gamma_var = (a5 * v_cmd^2) + (a6 * omega_cmd^2);

    % Sample final noisy orientation (gamma_hat)
    % Formula: gamma_hat = sample(gamma_var)
    % Sample noise from N(0, gamma_var)
    gamma_hat = randn * sqrt(gamma_var);

    % Calculate new pose based on noisy velocities
    % First, get the radius of the circular turn
    turn_radius = v_hat / omega_hat;

    % Calculate the next x position
    % Formula: x_next = x_prev - (v_hat/omega_hat)*sin(theta_prev) + (v_hat/omega_hat)*sin(theta_prev + omega_hat*dt)
    x_next = x_prev - turn_radius * sin(theta_prev) + turn_radius * sin(theta_prev + omega_hat * dt);

    % Calculate the next y position
    % Formula: y_next = y_prev + (v_hat/omega_hat)*cos(theta_prev) - (v_hat/omega_hat)*cos(theta_prev + omega_hat*dt)
    y_next = y_prev + turn_radius * cos(theta_prev) - turn_radius * cos(theta_prev + omega_hat * dt);
   
    % Calculate the next orientation
    % Formula: theta_next = theta_prev + omega_hat * dt + gamma_hat * dt
    theta_next = theta_prev + omega_hat * dt + gamma_hat * dt;

    % Final sampled pose vector xt = [x_next; y_next; theta_next].
    xt = [x_next; y_next; theta_next];
end 