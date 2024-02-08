function control_variables = load_control_parameters()
    % These variables should be similar to the ones you have in your
    % Arduino control software
    
    % Keep track of piston state, time it was last changed and for how long
    % the piston should be be on and off

    control_variables.piston_time_on = 0.2; % (seconds) [SELECT] Period the piston is on
    control_variables.piston_time_off = 0.5; % (seconds) [SELECT] Period the piston is off
    control_variables.piston_state = 0; % current piston state
    control_variables.piston_time_changed = -Inf; % last time we changed the piston state
    
    % Keep track of when we see ticks of the encoder (reed switch)
    control_variables.time_reed_switch = 0; % time reed switch has turned on
    control_variables.time_reed_switch_last = 0; % last time reed switch had turned on
    control_variables.reed_switch_last = 1; % reed switch state in the last iteration
    
    % Info about the course
    control_variables.grid_size = 66; % (inches) side length of sidewalk square grid
    
    % Set robot's initial conditions
    control_variables.starting_position = control_variables.grid_size*[-1.5 -1.75]; % (inches) [SELECT]
    control_variables.starting_direction = 90 * pi/180; % (radians) [SELECT]
    control_variables.starting_servo_angle = 0; % (radians) [SELECT]
    
    % Define variables for robot position and direction estimation
    control_variables.magnetometer_weight = 0.5; % [SELECT] set to 0 to not use magnetometer, 1 to only use magnetometer (no estimation)
    control_variables.position_estimate = control_variables.starting_position; % x and y
    control_variables.speed_estimate = 0;
    control_variables.robot_direction_estimate = control_variables.starting_direction;
    
    % Set a variable that will define the desired angle of the servo motor
    % for steering and the gains for the controller
    control_variables.servo_angle_desired = control_variables.starting_servo_angle;
    
    % Select type of controller:
    % 1 - Open-loop control: uses number of ticks to decide when to
    % start turning, and time to decide when to stop turning. No sensors used.
    % 2 - Hybrid control: uses ticks to decide when to start turning, and
    % time to decide when to stop turning. After that, it uses the
    % magnetometer to try to keep the correct direction
    % 3 - Closed-loop control: uses a via-point strategy with dead
    % reckoning as well as the magnetometer to control the robot
    control_variables.control_type = 1;
    
    % Variables for open-loop control
    control_variables.c1_n_ticks_before_turn = 25; %[SELECT] number of ticks before start turning
    control_variables.c1_total_time_turning = 0.28; %[SELECT] seconds while robot will be turning at max rate
    control_variables.c1_n_ticks = 0; % count number of ticks
    control_variables.c1_time_started_turning = 0; % saves when the robot started turning
    control_variables.c1_state = 0; % saves the current control state (approaching channel, turning, or in the channel)
    
    % Variables for hybrid control
    control_variables.c2_n_ticks_before_turn = 12; %[SELECT] number of ticks before start turning
    control_variables.c2_total_time_turning = 0.5; %[SELECT] seconds while robot will be turning at max rate
    control_variables.c2_Kp = 0.8; % [SELECT] causes the robot to point toward the channel
    control_variables.c2_n_ticks = 0; % count number of ticks
    control_variables.c2_time_started_turning = 0; % saves when the robot started turning
    control_variables.c2_state = 0; % saves the current control state (approaching channel, turning, or in the channel)
    
    % Define variables to control the targets in the course

    control_variables.c3_Kp = 0.8; % [SELECT] causes the robot to point toward the target
    control_variables.c3_servo_angle_max = 40 * pi/180; % [MEASURE] Max angle (+/- from straight) you want to send the servo motor to
    control_variables.c3_change_target_distance = 12; % (inches) [SELECT] minimum distance from current target to go to next target
    control_variables.c3_position_desired_list = control_variables.grid_size*[-1.5 0;
                                                                               4   0;
                                                                               8.5 0]; % [SELECT] each row is a target position (x,y)
    control_variables.c3_current_target = 1; % store current target number
    control_variables.c3_servo_angle_delta_max = 2; % 
end