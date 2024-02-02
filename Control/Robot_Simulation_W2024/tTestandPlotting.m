load("openLoopScore.mat");
load("hybridLoopScore.mat");
load("closedLoopScore.mat");

average = zeros(3,1);
average(1) = mean(openLoopScore);
average(2) = mean(hybridLoopScore);
average(3) = mean(closedLoopScore);

deviation = zeros(3,1);
deviation(1) = std(openLoopScore);
deviation(2) = std(hybridLoopScore);
deviation(3) = std(closedLoopScore);

[OvHdecision, OvHp] = ttest2(openLoopScore, hybridLoopScore);
[OvCdecision, OvCp] = ttest2(openLoopScore, closedLoopScore);
[HvCdecision, HvCp] = ttest2(hybridLoopScore, closedLoopScore);

figure(1);
hold on;

bar(average);
errorbar(average,deviation,'.');
XTickLabels={'Open-Loop'; 'Hybrid-Loop'; 'Closed-Loop'};
XTick=1:1:3;
set(gca, 'XTick',XTick);
set(gca, 'XTickLabel', XTickLabels);

