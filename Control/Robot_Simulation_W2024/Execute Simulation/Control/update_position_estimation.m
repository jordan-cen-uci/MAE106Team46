function sim = update_position_estimation(sim, magnetometer, reed_switch)
    %% Load struct variables into local variables
    servo_angle_desired         = sim.vars.servo_angle_desired;
    position_estimate           = sim.vars.position_estimate;
    robot_direction_estimate   	= sim.vars.robot_direction_estimate;
    speed_estimate              = sim.vars.speed_estimate;
    reed_switch_last            = sim.vars.reed_switch_last;
    time_reed_switch            = sim.vars.time_reed_switch;
    time_now                    = sim.current_time;
    delta_s_w                   = sim.robot.delta_s_w;
    magnetometer_weight         = sim.vars.magnetometer_weight;
    b                           = sim.robot.b;
    delta_s                     = sim.robot.delta_s_w;
    time_reed_switch_array      = sim.vars.time_reed_switch_array;
    
    %% Main logic
    % update estimator whenever reed switch activates
    if (reed_switch == 1 && reed_switch_last == 0)   
        % calculate avereage speed since last update
        speed_estimate = delta_s_w/(time_now-time_reed_switch);
        time_reed_switch = time_now;
        time_reed_switch_array = [time_reed_switch_array; time_reed_switch];
        robot_direction_estimate_last = robot_direction_estimate;
               
        % model steering kinematics
        if servo_angle_desired == 0
            
            % robot_direction_estimate = robot_direction_estimate;
            position_estimate(1) = position_estimate(1) + delta_s*cos(robot_direction_estimate); 
            position_estimate(2) = position_estimate(2) + delta_s*sin(robot_direction_estimate);
        else
            r_servo_angle = b/tan(servo_angle_desired);

            % [CALCULATE] if using a different reed switch placement (e.g front of 4-wheel design) enter relationship between wheel and robot distance travelled here:
            % delta_s = r_servo_angle/(r_servo_angle-a/2) * delta_s_w; % use this line for left wheel reed switch
            delta_s = cos(servo_angle_desired) * delta_s_w; % use this line for front wheel reed switch (tricycle design)
            robot_direction_estimate = wrapToPi(robot_direction_estimate + delta_s/r_servo_angle);        
            position_estimate(1) = position_estimate(1) + r_servo_angle*(sin(robot_direction_estimate)-sin(robot_direction_estimate_last));
            position_estimate(2) = position_estimate(2) - r_servo_angle*(cos(robot_direction_estimate)-cos(robot_direction_estimate_last));
        end
        % average robot_direction_estimate with magnetometer reading to correct for drift
        robot_direction_estimate = wrapToPi(robot_direction_estimate + magnetometer_weight*wrapToPi(magnetometer-robot_direction_estimate));
        
    end
    
    %% Save variables back into struct
    % update the variables we changed back in the struct
    sim.vars.position_estimate          = position_estimate;
    sim.vars.robot_direction_estimate   = robot_direction_estimate;
    sim.vars.speed_estimate             = speed_estimate;
    sim.vars.time_reed_switch           = time_reed_switch;
    sim.vars.time_reed_switch_array     = time_reed_switch_array;
end
    