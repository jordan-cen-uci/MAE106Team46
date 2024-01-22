function [sim, servo_angle_desired] = control_steering_closed_loop(sim, magnetometer, reed_switch)
%% Load struct variables into local variables
% simulation variables
dt                          = sim.dt;

% closed-loop control variables
position_estimate           = sim.vars.position_estimate;
robot_direction_estimate    = sim.vars.robot_direction_estimate;
servo_angle_desired         = sim.vars.servo_angle_desired;
current_target              = sim.vars.c3_current_target;
position_desired_list       = sim.vars.c3_position_desired_list;
Kp                          = sim.vars.c3_Kp;
change_target_distance      = sim.vars.c3_change_target_distance;
servo_angle_max             = sim.vars.c3_servo_angle_max;
servo_angle_delta_max       = sim.vars.c3_servo_angle_delta_max;

%% Main logic
% determine whether to apply channel control, based on settings and robot position
position_desired = position_desired_list(current_target,:);

% calculate useful errors:
position_error = position_estimate - position_desired; % vector error between robot and target positions
position_error_magnitude = sqrt(sum(position_error.^2)); % distance between robot and target

robot_to_target_angle = wrapToPi(atan2(-position_error(2),-position_error(1))); % direction from robot to target
direction_to_target_error = wrapToPi(robot_to_target_angle - robot_direction_estimate); % error between orientation and that angle

% steering control law
% simple steering control law:
servo_angle_target = Kp * direction_to_target_error;
servo_angle_target = sign(servo_angle_target)*min(servo_angle_max,abs(servo_angle_target)); % limit steering to feasible range

% limits the rate-of-change of servo_angle_desired so as not to exceed that of servo motor
delta_servo_angle_desired = servo_angle_target-servo_angle_desired;

servo_angle_desired = servo_angle_desired + sign(delta_servo_angle_desired)*min(dt*servo_angle_delta_max,abs(delta_servo_angle_desired));
if(abs(servo_angle_desired) > servo_angle_max)
    servo_angle_desired = sign(servo_angle_desired)*servo_angle_max;
end


if position_error_magnitude < change_target_distance % if it's close enough to current target, move to the next target
    if(current_target < length(position_desired_list)) % go to next target unless already at the last target
        current_target = current_target + 1;
    end
end

%% Store variables back into main struct
sim.vars.c3_current_target          = current_target;
sim.vars.servo_angle_desired        = servo_angle_desired;
sim.position_desired(sim.n, :)      = position_desired;
end

