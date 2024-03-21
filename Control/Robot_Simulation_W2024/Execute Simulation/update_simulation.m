function sim = update_simulation(sim)
% simulation step
n = sim.n;
% get variables into our main struct for future reference and plot

% simulation parameters
time                    = sim.time;
time_now                = sim.time(n);
dt                      = sim.dt;

% robot parameters
servo_angle_delta_max   = sim.robot.servo_angle_delta_max;
servo_angle_max         = sim.robot.servo_angle_max;
b                       = sim.robot.b;
delta_s_w               = sim.robot.delta_s_w;

% main simulation parameters
servo_angle_desired     = sim.servo_angle_desired;
servo_angle_error       = sim.servo_angle_error;
servo_angle_last        = sim.servo_angle(n-1);
piston_state            = sim.piston_state;
piston_state_last       = sim.piston_state_list(n-1);
piston_fired            = sim.piston_fired;
piston_fired_time       = sim.piston_fired_time;
piston_fired_attempt    = sim.piston_fired_attempt;
speed                   = sim.speed;
impulse_response        = sim.impulse_response.average_response;
wheel_distance          = sim.wheel_distance;
wheel_encoder           = sim.wheel_encoder;
robot_direction         = sim.robot_direction;
position                = sim.position;
decay_response          = sim.impulse_response.decay;
magnetometer_noise      = sim.magnetometer_noise;
grid_size               = sim.grid_size;
stop_score              = sim.stop_score;
last_score              = sim.score(n-1);


% check if there's a command for actuate the piston
if(piston_state == 1 && piston_state_last == 0)
    piston_fired_attempt(n) = 1;
else
    piston_fired_attempt(n) = 0;
end

% add impulse response to robot speed each time the piston fires, taking
% into account the decay over the number of actuations
if piston_fired_attempt(n)
    decay = decay_response(sum(piston_fired_attempt));
    if ((time_now - piston_fired_time) > sim.robot.time_between_piston_fires_min)
        piston_fired_time = time_now;
        piston_fired(n) = 1;
        for m = 0:(length(time)-n)
            speed(n+m) = speed(n+m) + impulse_response(m+1)*decay;
        end
    end
end

% steering mechanism dynamics
delta_servo_angle = (servo_angle_desired+servo_angle_error(time_now))-servo_angle_last;
servo_angle = servo_angle_last + sign(delta_servo_angle)*min(dt*servo_angle_delta_max,abs(delta_servo_angle));
servo_angle = sign(servo_angle) * min(servo_angle_max,abs(servo_angle));

% model steering kinematics
if servo_angle == 0
    robot_direction(n) = robot_direction(n-1);
    position(n,1) = position(n-1,1) + dt*speed(n)*cos(robot_direction(n));
    position(n,2) = position(n-1,2) + dt*speed(n)*sin(robot_direction(n));
    wheel_distance(n) = wheel_distance(n-1) + dt*speed(n);
else
    robot_direction(n) = wrapToPi(robot_direction(n-1) + dt*speed(n)*sin(servo_angle)/b);
    position(n,1) = position(n-1,1) + b/sin(servo_angle)*(sin(robot_direction(n))-sin(robot_direction(n-1)));
    position(n,2) = position(n-1,2) - b/sin(servo_angle)*(cos(robot_direction(n))-cos(robot_direction(n-1)));
    
    % [CALCULATE] if using a different reed switch placement (e.g front of 4-wheel design) enter relationship between wheel and robot distance travelled here:
    %        wheel_distance(n) = wheel_distance(n-1) + dt*speed(n)*(1-a/(2*b)*sin(servo_angle)); % use this formula for left wheel reed switch
    wheel_distance(n) = wheel_distance(n-1) + dt*speed(n)/cos(servo_angle); % use this formula for front wheel reed switch (tricycle design)
end

if stop_score
    current_score = last_score;
else
    if (position(n-1,1) < 0 && position(n,1)>0) && ( abs(position(n, 2)) > grid_size/2 )
        current_score = 0;
        stop_score = 1;
    elseif (position(n,1) < -3 * grid_size)
        current_score = 0;
        stop_score = 1;
    elseif (abs(position(n,2)) > 2.5 * grid_size)
        current_score = 0;
        stop_score = 1;
    elseif position(n, 1) > -2 * grid_size
        in_channel = (abs(position(n, 2)) < grid_size/2);
        current_score = 10 * ceil( (position(n,1) + 2*grid_size)  / grid_size);
        if ~in_channel
            if current_score >= 30
                current_score = current_score*0.5;
            else
                current_score = 0;
            end
        end
        
        % crossed final line
        if (position(n-1,1) < 8*grid_size && position(n,1)>= 8*grid_size)
           if(in_channel)
               current_score = 150;
               stop_score = 1;
           else
               current_score = 50;
               stop_score = 1;
           end
        end
        
    end
end


distance_to_switch = abs( (wheel_distance(n)/delta_s_w) - round((wheel_distance(n)/delta_s_w)));
if n > 7
    wheel_encoder(n) = distance_to_switch < 0.1;
end

% update the variables available for control
sim.reed_switch = wheel_encoder(n);
sim.magnetometer = wrapToPi(robot_direction(n)+pi/180*magnetometer_noise*randn);

% update values in main struct
sim.servo_angle_desired_list(n)     = servo_angle_desired;
sim.servo_angle(n)                  = servo_angle;
sim.piston_state_list(n)            = piston_state;
sim.speed                           = speed;
sim.wheel_distance                  = wheel_distance;
sim.wheel_encoder                   = wheel_encoder;
sim.robot_direction                 = robot_direction;
sim.position                        = position;
sim.piston_fired                    = piston_fired;
sim.piston_fired_time               = piston_fired_time;
sim.piston_fired_attempt            = piston_fired_attempt;
sim.stop_score                      = stop_score;
sim.position_estimate(n, :)         = sim.vars.position_estimate;
sim.robot_direction_estimate(n)     = sim.vars.robot_direction_estimate;
sim.robot_direction_magnetometer(n) = sim.magnetometer;
sim.speed_estimate(n)               = sim.vars.speed_estimate;
sim.score(n)                        = current_score;

% update simulation parameters
sim.n = n + 1;
if(sim.n <= length(time))
    sim.current_time = time(sim.n);
end