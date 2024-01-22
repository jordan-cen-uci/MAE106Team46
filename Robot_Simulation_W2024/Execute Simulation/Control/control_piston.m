function [sim, piston_state] = control_piston(sim, magnetometer, reed_switch)
%% Load struct variables into local variables
piston_time_changed         = sim.vars.piston_time_changed;
piston_state                = sim.vars.piston_state;
piston_time_on              = sim.vars.piston_time_on;
piston_time_off             = sim.vars.piston_time_off;
time_now                    = sim.current_time;

%% Main logic
if(piston_state == 1)
    if( (time_now - piston_time_changed) > piston_time_on )
        piston_state = 0;
        piston_time_changed = time_now;
    end
else
    if( (time_now - piston_time_changed) > piston_time_off )
        piston_state = 1;
        piston_time_changed = time_now;
    end
end

%% Save variables back into struct
% update the variables we changed back in the struct
sim.vars.piston_time_changed   = piston_time_changed;
end
