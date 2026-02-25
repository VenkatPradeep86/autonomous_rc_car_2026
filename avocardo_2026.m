% --- OPTIMIZED AUTONOMOUS RUN: BIAS CORRECTION ---
setenv('ROS_DOMAIN_ID', '30');
clear node;
node = ros2node("/optimized_driver", 30);

sub = ros2subscriber(node, "/yolo/simple_detections", "std_msgs/Float32MultiArray", 'Reliability', 'besteffort');
pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
drive_msg = ros2message(pub);

% --- TUNING PARAMETERS ---
CENTER_X = 690;     
KP = -0.0018;       
BASE_SPEED = 1.0;   

STEERING_TRIM = -0.1; 

fprintf('!!! RUNNING WITH TRIM CORRECTION: %.2f !!!\n', STEERING_TRIM);

while true
    [msg, status] = receive(sub, 0.5);
    if status && ~isempty(msg.data)
        data = msg.data;
        bx = []; by_max = -1;
        yx = []; yy_max = -1;

        for j = 1:3:numel(data)
            if data(j+2) == 0 && data(j+1) > by_max, bx = data(j); by_max = data(j+1); end
            if data(j+2) == 1 && data(j+1) > yy_max, yx = data(j); yy_max = data(j+1); end
        end

        if ~isempty(bx) && ~isempty(yx)
            midpoint = (double(bx) + double(yx)) / 2;
            err_val = midpoint - double(CENTER_X);

            % CALCULATE STEERING WITH TRIM
            raw_steering = err_val * KP;
            
            % Final steering is the error correction + the hardware offset fix
            drive_msg.angular.z = double(raw_steering + STEERING_TRIM);
            drive_msg.linear.x = double(BASE_SPEED);
            
            send(pub, drive_msg);
            fprintf('Mid: %.1f | Err: %.1f | Final Steer: %.3f\n', midpoint, err_val, drive_msg.angular.z);
        else
            drive_msg.linear.x = 0.0;
            drive_msg.angular.z = 0.0;
            send(pub, drive_msg);
            fprintf('!!! TRACK LOST !!!\n');
        end
    end
end