function params = load_simulation_parameters()
    % This function loads the parameters for your simulation. 
    params.sampling_frequency = 100; % Frequency in which the simulation is running (in hertz)
    params.total_time = 60; % For how long to run the simulation (in seconds)
    
    % Animation parameters
    params.view_style = 1; % [SELECT] 1 = course view, 2 = robot view
    params.playback_speed = 5; % [SELECT] what multiple of real-time to play simulation
    params.fps = 30; % [SELECT] approximate the fps of the animation on your computer
end
