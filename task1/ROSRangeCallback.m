% Filename: ROSRangeCallback.m
% Author: Filip Hanuš
% Description: Callback for LiDAR scan subscriber to update global range readings (front, left, right).

function ROSRangeCallback(~, message)
    % Global same as in the controller
    global GL_ranges
    
    % Extract lidar range in front (+/- 10 degrees)
    front_ranges = [message.Ranges(350:360); message.Ranges(1:10)];
    GL_ranges(1) = min(front_ranges); % Find minimum, ignore 0 ranges
    
    % Extract lidar range on the left (90 +/- 10 degrees)
    left_ranges = message.Ranges(80:100);
    GL_ranges(2) = min(left_ranges);
    
    % Extract lidar range on the right (270 +/- 10)
    right_ranges = message.Ranges(260:280);
    GL_ranges(3) = min(right_ranges);
end