function [sim, servo_angle_desired] = control_steering_open_loop(sim, magnetometer, reed_switch)
%% Load struct variables into local variables
% simulation variables
time_now                = sim.current_time;

% open loop control variables
n_ticks                 = sim.vars.c1_n_ticks;
n_ticks_before_turn     = sim.vars.c1_n_ticks_before_turn;
total_time_turning      = sim.vars.c1_total_time_turning;
time_started_turning    = sim.vars.c1_time_started_turning;
state                   = sim.vars.c1_state;
reed_switch_last        = sim.vars.reed_switch_last;

%% Main logic

% 3 states:
% state = 0 is approaching the channel
% state = 1 is turning into the channel
% state = 3 is to keep servo straight through the channel

if state == 0
    % approaching channel, count number of ticks
    if (reed_switch == 1 && reed_switch_last == 0)   
         n_ticks = n_ticks + 1;
    end
    % if we had enough ticks, change state to start turning
    if (n_ticks >= n_ticks_before_turn)
        state = 1;
        time_started_turning = time_now;
    end
    servo_angle_desired = 0;
elseif state == 1
    % check if we have turned for enough time, if so, set servo back
    % straight
    servo_angle_desired = -90 * pi / 180;
    if ((time_now - time_started_turning) > total_time_turning)
        state = 2;
    end
else
    servo_angle_desired = 0;
end
        
%% Store variables back into main struct
sim.vars.servo_angle_desired       = servo_angle_desired;
sim.vars.c1_n_ticks                = n_ticks;
sim.vars.c1_n_ticks_before_turn    = n_ticks_before_turn;
sim.vars.c1_total_time_turning     = total_time_turning;
sim.vars.c1_time_started_turning   = time_started_turning;
sim.vars.c1_state                  = state;

end

