function impulse_response = load_impulse_response(robot, simulation)
    %% Impulse Response (velocity profile created by one pulse)
    % [EXPERIMENT] Enter your data into impulse_response.data, each row is for one trial, add NaN as necessary to make rectangular matrix
    % Each of the points in the impulse_response.data is the time of a tick
    % on your encoder.
    impulse_response.data = [.15 .25 .40 .9  1.4  2.8 NaN; ...
                             .14 .26 .41 .67 1.42 2.9 5.1];
                                
    impulse_response.scale = 1.5; % [EXPERIMENT] use to calibrate your max speed
    impulse_response.decay = @(n) max(0,polyval([-.00025 -.0038 1.0103],n)); % [EXPERIMENT] This equates how much less movement your robot gets with each actuation of the piston. This happens due to the reduced pressure in the tire.
    
    % calculate the average impulse response based on your data
    t = (0:1/simulation.sampling_frequency:simulation.total_time)';
    empty_vector = zeros([length(t), 1]);
    impulse_response_M = repmat(empty_vector,[1 size(impulse_response.data,1)]);
    for n = 1:size(impulse_response.data,1)
        IR_ticks = [0 impulse_response.data(n,~isnan(impulse_response.data(n,:)))];
        IR_time = [0 IR_ticks(1:end-1)+diff(IR_ticks)/2 IR_ticks(end) 100];
        IR_speed = [0 robot.delta_s_w./diff(IR_ticks) 0 0];
        impulse_response_M(:,n) = smooth(interp1(IR_time,IR_speed, t),.25 * simulation.sampling_frequency);
    end

    % average results across trials to create the best estimate of the true impulse response
    impulse_response.average_response = impulse_response.scale*mean(impulse_response_M,2);
    
end