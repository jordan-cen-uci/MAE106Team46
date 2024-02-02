addpath('Setup Simulation');
addpath('Execute Simulation');
addpath('Execute Simulation/Control');
addpath('Plot');

% load parameters
robot = load_physical_parameters();
control = load_control_parameters();
simulation_params = load_simulation_parameters();
impulse_response = load_impulse_response(robot, simulation_params);

% initialize variables
sim = initialize_simulation_data(robot, control, impulse_response, simulation_params);

% go through the simulation and control the robot
for n = 2:length(sim.time)
    sim = update_control(sim);
    sim = update_simulation(sim);
end

% plot results and animation
%plot_animation(sim);
%plot_performance(sim);
%plot_impulse_response(sim);