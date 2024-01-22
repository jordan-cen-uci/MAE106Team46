function plot_performance(sim)
%% Plot Results

time = sim.time;
dt = sim.dt;

% Performance Graphs
figure(2)
clf

subplot(311)
speed_estimate = sim.speed_estimate;
speed_actual = sim.speed;

plot(time,[speed_estimate speed_actual],'LineWidth',1)
title('Robot Speed')
xlabel('Time (seconds)')
ylabel('Speed (inches/seconds)')
legend('Estimate','Actual')

subplot(312)
servo_angle_desired = sim.servo_angle_desired_list;
servo_angle_actual = sim.servo_angle;
servo_angle_max = sim.robot.servo_angle_max;
plot(time,[servo_angle_desired servo_angle_actual]*180/pi)
axis([0 time(end) [-1.1 1.1] * servo_angle_max*180/pi])
title('Steering Angle')
xlabel('Time (seconds)')
ylabel('servo_angle (degrees)')
legend('Estimate','Actual')

subplot(313)
servo_angle_error = sim.servo_angle_error;
plot(time,servo_angle_error(time)*180/pi)
title('Steering Error')
xlabel('time (seconds)')
ylabel('servo_angle error (degrees)')
