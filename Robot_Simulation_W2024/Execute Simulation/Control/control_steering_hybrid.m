function [sim, servo_angle_desired] = control_steering_hybrid(sim, magnetometer, reed_switch)
%% Load struct variables into local variables
% simulation variables
time_now                = sim.current_time;

% hybrid loop control variables
n_ticks                 = sim.vars.c2_n_ticks;
n_ticks_before_turn     = sim.vars.c2_n_ticks_before_turn;
total_time_turning      = sim.vars.c2_total_time_turning;
time_started_turning    = sim.vars.c2_time_started_turning;
state                   = sim.vars.c2_state;
reed_switch_last        = sim.vars.reed_switch_last;
Kp                      = sim.vars.c2_Kp;

%% Main logic

% 3 states:
% state = 0 is approaching the channel
% state = 1 is turning into the channel
% state = 3 is to keep servo straight through the channel using the
% magnetometer to feedback angle

if state == 0
    % count the number of ticks in the reed switch
    if (reed_switch == 1 && reed_switch_last == 0)   
         n_ticks = n_ticks + 1;
    end
    
    % check if we have enough ticks to start turning
    if (n_ticks >= n_ticks_before_turn)
        state = 2;
        time_started_turning = time_now;
    end
    servo_angle_desired = -Kp * (magnetometer - pi/2);
    %servo_angle_desired = 0;
elseif state == 1
    % set servo to the max angle during turning time
    servo_angle_desired = -90 * pi / 180;
    
    % check if we've turned for long enough
    if ((time_now - time_started_turning) > total_time_turning)
        state = 2;
    end
elseif state == 2
    % try to change the servo such that magnetometer reads 0 (in this case,
    % magnetometer = 0 is the direction down the channel)
    servo_angle_desired = -Kp * magnetometer;
end           
   
%% Store variables back into main struct
sim.vars.servo_angle_desired       = servo_angle_desired;

sim.vars.c2_n_ticks                = n_ticks;
sim.vars.c2_n_ticks_before_turn    = n_ticks_before_turn;
sim.vars.c2_total_time_turning     = total_time_turning;
sim.vars.c2_time_started_turning   = time_started_turning;
sim.vars.c2_state                  = state;

end

