function plot_impulse_response(sim)

time = sim.time;
dt = sim.dt;
impulse_response = sim.impulse_response.average_response;
delta_s_w = sim.robot.delta_s_w;
IR_data = sim.impulse_response.data;

% Graphs to Troubleshoot Impulse Response
figure(3)
clf

subplot(211)
hold on


test_pos = cumsum(impulse_response*dt);
IR_pos = repmat(delta_s_w*(1:size(IR_data,2)),[size(IR_data,1) 1]);
plot(IR_data,IR_pos,'k.')
ax1 = axis;
plot(time,test_pos)
axis([ax1(1:2) 0 ax1(4)])
title('Impulse Response')
xlabel('time (seconds)')
ylabel('position (inches)')

subplot(212)
plot(time,impulse_response)
ax2 = axis;
axis([ax1(1:2) ax2(3:4)])
title('Impulse Response')
xlabel('time (seconds)')
ylabel('velocity (inches/second)')