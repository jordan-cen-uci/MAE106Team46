part1 = readtable("Lab 5 - Part 1 Data.csv");
part2 = readtable("Lab 5 - Part 2 Data.csv");

part1UnderDampTime = part1.x_RiseTime_(6:end);
part1UnderDampPos = part1.Var15(6:end);
part1UnderDampDes = part1.Var16(6:end);

part1CriticalTime = part1.GoatedTrial(6:end);
part1CriticalPos = part1.Var27(6:end);
part1CriticalDes = part1.Var28(6:end);

part1OverDampTime = part1.Var46(6:end);
part1OverDampPos = part1.Var47(6:end);
part1OverDampDes = part1.Var48(6:end);

figure(1);
hold on;
plot(part1UnderDampTime, part1UnderDampPos,'b-', part1UnderDampTime, part1UnderDampDes, 'r-');
ax1 = gca;
ax1.XLim = [0 0.2];
ax1.YLim = [0 1.1];
title('Underdamped Trial; Kp = 90, Kd = 0');
legend('Actual Position', 'Desired Position','Location','southeast');
xlabel('Time (s)');
ylabel('Position (radians)');
xticks([0 0.05 0.10 0.15 0.2]);

figure(2);
hold on;
plot(part1CriticalTime, part1CriticalPos,'b-', part1CriticalTime, part1CriticalDes, 'r-');
ax2 = gca;
ax2.XLim = [0 0.2];
ax2.YLim = [0 1.1];
title('Underdamped Trial; Kp = 90, Kd = 0.2');
legend('Actual Position', 'Desired Position','Location','southeast');
xlabel('Time (s)');
ylabel('Position (radians)');
xticks([0 0.05 0.10 0.15 0.2]);

figure(3);
hold on;
plot(part1OverDampTime, part1OverDampPos,'b-', part1OverDampTime, part1OverDampDes, 'r-');
ax3 = gca;
ax3.XLim = [0 0.2];
ax3.YLim = [0 1.1];
title('Underdamped Trial; Kp = 90, Kd = 4.5');
legend('Actual Position', 'Desired Position','Location','southeast');
xlabel('Time (s)');
ylabel('Position (radians)');
xticks([0 0.05 0.10 0.15 0.2]);

part2UndampedFreq = [1;2;4;5;6;8;12;16];
part2UndampedOutput = [1;1;1;1;0.9995;0.999;1;1];
part2UndampedInput = [0.941;0.907;0.995;0.9075;0.812;0.494;0.3385;0.2235];

part2UndampedTrans = part2UndampedInput./part2UndampedOutput;

part2UndampedAmp = 20.*log10(part2UndampedTrans);

part2UndampedPhase = [];

figure(4);
clf;
semilogx(part2UndampedFreq, part2UndampedAmp);
hold on;

part2DampedFreq = [1;2;4;5;6;8;12;16];
part2DampedOutput = [1;1;0.9995;1;1;0.999;0.9995;1];
part2DampedInput = [0.7445;0.894;0.9005;0.88;0.7855;0.5755;0.2435;0.2775];

part2DampedTrans = part2DampedInput./part2DampedOutput;

part2DampedAmp = 20.*log10(part2DampedTrans);

figure(5);
clf;
semilogx(part2DampedFreq, part2DampedAmp);
