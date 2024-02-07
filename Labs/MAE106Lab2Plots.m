part1Type1 = readtable("Lab2_Part1 Type1 Data.csv");
part1Type2 = readtable("Lab2_Part1 Type2 Data.csv");
part1Type3 = readtable("Lab2_Part1 Type3 Data.csv");

part2Type1 = readtable("Lab2Part2Type1.csv");
part2Type2 = readtable("Lab2Part2Type2.csv");
part2Type3 = readtable("Lab2Part2Type3.csv");

part1Type1InputX = part1Type1.Var4;
part1Type1InputY = part1Type1.Var5;
part1Type1FilterX = part1Type1.Var10;
part1Type1FilterY = part1Type1.Var11;

part1Type2InputX = part1Type2.Var4;
part1Type2InputY = part1Type2.Var5;
part1Type2FilterX = part1Type2.Var10;
part1Type2FilterY = part1Type2.Var11;

part1Type3InputX = part1Type3.Var4;
part1Type3InputY = part1Type3.Var5;
part1Type3FilterX = part1Type3.Var10;
part1Type3FilterY = part1Type3.Var11;

part2Type1Time = part2Type1.Var1;
part2Type1Input = part2Type1.Var2;
part2Type1Filter = part2Type1.Var4;

part2Type2Time = part2Type2.Var1;
part2Type2Input = part2Type2.Var2;
part2Type2Filter = part2Type2.Var4;

part2Type3Time = part2Type3.Var1(20: 110);
part2Type3Input = part2Type3.Var2(20: 110);
part2Type3Filter = part2Type3.Var4(20: 110);



figure(1);
hold on;

plot(part1Type1InputX, part1Type1InputY, part1Type1FilterX, part1Type1FilterY);
title("Analog Low Pass Filter, Case 1: Filter Off");
legend('Input Signal', 'Filtered Signal');
xlabel('Time (ms)');
ylabel('Signal Voltage (V)');

figure(2);
hold on;

plot(part1Type2InputX, part1Type2InputY, part1Type2FilterX, part1Type2FilterY);
title("Analog Low Pass Filter, Case 2: Filter Completely On");
legend('Input Signal', 'Filtered Signal');
xlabel('Time (ms)');
ylabel('Signal Voltage (V)');

figure(3);
hold on;

plot(part1Type3InputX, part1Type3InputY, part1Type3FilterX, part1Type3FilterY);
title("Analog Low Pass Filter, Case 3: Filter Partially On");
legend('Input Signal', 'Filtered Signal');
xlabel('Time (ms)');
ylabel('Signal Voltage (V)');

figure(4);
hold on;

plot(part2Type1Time, part2Type1Input, part2Type1Time, part2Type1Filter);
title("Digital Low Pass Filter, Case 1: Filter Off");
legend('Input Signal', 'Filtered Signal');
xlabel('Time (ms)');
ylabel('Signal Percentage (%)');
ax4 = gca;
ax4.XLim = [22990 23050];
ax4.YLim = [-0.2 1.2];

figure(5);
hold on;

plot(part2Type2Time, part2Type2Input, part2Type2Time, part2Type2Filter);
title("Digital Low Pass Filter, Case 2: Filter Completely On");
legend('Input Signal', 'Filtered Signal');
xlabel('Time (ms)');
ylabel('Signal Percentage (%)');
ax5 = gca;
ax5.XLim = [3090 3150];
ax5.YLim = [-0.2 1.2];

figure(6);
hold on;

plot(part2Type3Time, part2Type3Input, part2Type3Time, part2Type3Filter);
title("Digital Low Pass Filter, Case 3: Filter 70% Duty Cycle");
legend('Input Signal', 'Filtered Signal');
xlabel('Time (ms)');
ylabel('Signal Percentage (%)');
ax6 = gca;
ax6.XLim = [30 90];
ax6.YLim = [-0.2 1.2];




