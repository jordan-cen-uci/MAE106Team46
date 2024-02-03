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



figure(1);
hold on;

plot(part1Type1InputX, part1Type1InputY, part1Type1FilterX, part1Type1FilterY);

figure(2);
hold on;

plot(part1Type2InputX, part1Type2InputY, part1Type2FilterX, part1Type2FilterY);

figure(3);
hold on;

plot(part1Type3InputX, part1Type3InputY, part1Type3FilterX, part1Type3FilterY);

figure(4);
hold on;

plot(part2Type1Time, part2Type1Input, part2Type1Time, part2Type1Filter);

ax4 = gca;
ax4.XLim = [22980 23040];

figure(5);
hold on;

plot(part2Type2Time, part2Type2Input, part2Type2Time, part2Type2Filter);




