% Filename: ROSPose_GazeboCallback.m
% Author: Filip Hanuš
% Description: Callback for Gazebo odometry subscriber to update global robot pose.

function ROSPose_GazeboCallback(~, message)
    % Global as in the controller to pass it the pos and orientation
    global GL_robot_pose
    
    % Extract 2D position (X, Y)
    pos_x = message.Pose.Pose.Position.X;
    pos_y = message.Pose.Pose.Position.Y;
    
    % Extract orientation quaternion {X Y Z W}
    quant = [message.Pose.Pose.Orientation.X, ...
            message.Pose.Pose.Orientation.Y, ...
            message.Pose.Pose.Orientation.Z, ...
            message.Pose.Pose.Orientation.W];
            
    % Convert quaternion to Euler angles [roll, pitch, yaw]
    eula = quat2eul(quant);
    
    % Extract yaw angle (theta, rotation around Z axis)
    theta = eula(3);
    
    % Store [x, y, theta] in the global variable
    GL_robot_pose = [pos_x, pos_y, theta];
end