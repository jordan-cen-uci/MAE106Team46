function simulation = initialize_simulation_data(robot, control, impulse_response, params)
    % This function initalizes all the variables need for the simulation 
    % Create time variables
    simulation.dt = 1/params.sampling_frequency;
    simulation.time = (0:simulation.dt:params.total_time)';
    
    % Create empty vector with the correct size to initialize other
    % variables
    empty_vector = zeros([length(simulation.time), 1]);
    
    % if the controller has a variable grid size, we'll use it. Otherwise,
    % we'll use the default of 66 inches.
    if(isfield(control, 'grid_size'))
        simulation.grid_size = control.grid_size;
    else
        simulation.grid_size = 66; % (inches) side length of sidewalk square grid
    end
    
    simulation.position = repmat(empty_vector,[1 2]); 
    simulation.speed = empty_vector;
    
    simulation.piston_state_list = empty_vector;
    simulation.piston_fired_time = -Inf;
    simulation.piston_fired = empty_vector; % this does NOT represent the signal sent to the piston,
                         % each 1 represents the beginning of a complete piston fire
    simulation.piston_fired_attempt = empty_vector;
    
    simulation.servo_angle = empty_vector;
    simulation.robot_direction = empty_vector;
    simulation.wheel_distance = empty_vector;
    simulation.wheel_encoder = empty_vector;

    % control inputs
    simulation.servo_angle_desired_list = empty_vector;

    % estimator values
    simulation.position_estimate = repmat(empty_vector,[1 2]);
    simulation.robot_direction_estimate = empty_vector;
    simulation.speed_estimate = empty_vector;

    % target values
    simulation.position_desired = repmat(empty_vector,[1 2]);
    simulation.robot_direction_desired = empty_vector;
    simulation.speed_desired = empty_vector;

    % keep track of magnetometer values (to see error)
    simulation.robot_direction_magnetometer = empty_vector;
    simulation.score = empty_vector;

    % impose inititial conditions. Use values from the controller if
    % present, otherwise use defaults.
    if(isfield(control, 'starting_postion'))
        simulation.position(1,:) = control.starting_postion;
        simulation.position_estimate(1,:) = control.starting_postion;
    else
        sp = simulation.grid_size*[-1.5 -1.75];
        simulation.position(1,:) = sp;
        simulation.position_estimate(1,:) = sp;
    end
    
    if(isfield(control, 'starting_direction'))
        simulation.robot_direction(1) = control.starting_direction;
        simulation.robot_direction_estimate(1) = control.starting_direction;
        simulation.robot_direction_magnetometer(1) = control.starting_direction;
    else
        sp = 90 * pi/180;
        simulation.robot_direction(1) = sp;
        simulation.robot_direction_estimate(1) = sp;
        simulation.robot_direction_magnetometer(1) = sp;
    end
        
    % error data
    simulation.magnetometer_noise = 5; % (degrees) adds a normally distributed error with this standard deviation
    simulation.servo_angle_error = 1; % (degrees) adds a normally distributed error with this standard deviation

    % create a drifting error in steering angle
    simulation.servo_angle_error_seed = randn([4 1]);
    simulation.servo_angle_error = @(t) simulation.servo_angle_error * (simulation.servo_angle_error_seed(1)*sin(2*pi*0.02*simulation.servo_angle_error_seed(2)*t)+...
    simulation.servo_angle_error_seed(3)*sin(2*pi*0.02*simulation.servo_angle_error_seed(4)*t))*pi/180;

    % create noise error on magnetometer reading
    simulation.magnetometer_error = @()pi/180*simulation.magnetometer_noise*randn;
    
    % variables available for control
    simulation.magnetometer = simulation.robot_direction(1)+simulation.magnetometer_error();
    simulation.reed_switch = 0;
    
    % simulation status
    simulation.n = 2;
    simulation.current_time = simulation.time(1);
    simulation.stop_score = 0;
    
    % save all the data into this struct
    simulation.robot = robot;
    simulation.vars = control;
    simulation.impulse_response = impulse_response;
    simulation.params = params;
end