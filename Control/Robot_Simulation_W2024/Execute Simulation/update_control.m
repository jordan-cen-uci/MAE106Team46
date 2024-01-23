function sim = update_control(sim) % main function
    % The only inputs you have in your robot come from the sensors.
    % That means, all of your control logic should only use the values
    % coming from the magnetometer and the reed switch.
    
    % Let's get our variables available for the control out of the
    % structure, so it's more similar to what you'll have in your Arduino
    % code
    magnetometer = sim.magnetometer;
    reed_switch = sim.reed_switch;
    
    % This should look similar to what you have in your main() function in
    % your Arduino code
    sim = update_position_estimation(sim, magnetometer, reed_switch);
    
    % load the control type we're using
    control_type = sim.vars.control_type;
    if(control_type == 1)
        [sim, servo_angle_desired] = control_steering_open_loop(sim, magnetometer, reed_switch);     
    elseif(control_type == 2)
        [sim, servo_angle_desired] = control_steering_hybrid(sim, magnetometer, reed_switch);     
    else
        [sim, servo_angle_desired] = control_steering_closed_loop(sim, magnetometer, reed_switch);     
    end
                
            
    [sim, piston_state] = control_piston(sim, magnetometer, reed_switch);
    sim.vars.reed_switch_last = reed_switch;

    % Your only way to interact with the world with your robot is changing
    % the steering angle and actuating the piston.
    % That means, your control logic should only change the values of 
    % servo_angle_desired and piston_state
    % Put the variables back into the main simulation struct
    sim.servo_angle_desired = servo_angle_desired;
    sim.piston_state = piston_state;
end