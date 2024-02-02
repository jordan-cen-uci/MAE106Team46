function robot = load_physical_parameters()
    % This function loads some of the phyisical properties of your robot.
    % You should measure or experiment with your robot to obtain some of
    % these parameters.
    
    robot.a = 7.28; % (inches) [MEASURE] distance between rear wheels
    robot.b = 5.58; % (inches) [MEASURE] distance between rear axle and front wheel
    robot.r_wheel = 1.375; % (inches) [MEASURE] roller blade wheel radius
    robot.num_magnets = 2; % [MEASURE] how many magnets are attached to the wheel (equally spaced)
    robot.servo_angle_delta_max = 2; % (rad/s) [EXPERIMENT] max rotational speed to steering mechanism
    robot.servo_angle_max = 45 * pi/180; % (radians) [EXPERIMENT] max rotation angle of steering mechanism

    robot.time_between_piston_fires_min = 0.3; % (seconds) the shortest allowable time between successive fires    
    robot.delta_s_w = 2*pi*robot.r_wheel/robot.num_magnets; % inches per reed switch update (must be consistent with impulse response data)
end

