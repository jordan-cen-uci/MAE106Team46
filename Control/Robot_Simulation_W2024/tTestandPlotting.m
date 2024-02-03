%loading all data from the twentyRun code
load("openLoopScore.mat");
load("hybridLoopScore.mat");
load("closedLoopScore.mat");

%initialize a matrix for the averages
average = zeros(3,1);

%assign the average of each run to an index of the average array
average(1) = mean(openLoopScore);
average(2) = mean(hybridLoopScore);
average(3) = mean(closedLoopScore);

%initialize a matrix for the standard deviations of each trial
deviation = zeros(3,1);

%assign the standard devaition to an index of the deviation array
deviation(1) = std(openLoopScore);
deviation(2) = std(hybridLoopScore);
deviation(3) = std(closedLoopScore);

%run ttest between Open vs Hybrid, Open vs Closed, and Hybrid vs Closed
[OvHdecision, OvHp] = ttest2(openLoopScore, hybridLoopScore);
[OvCdecision, OvCp] = ttest2(openLoopScore, closedLoopScore);
[HvCdecision, HvCp] = ttest2(hybridLoopScore, closedLoopScore);

figure(1);
hold on;

%plot bar plot of averages
bar(average);
%add standard deviation bars
errorbar(average,deviation,'.');
%add labels
XTickLabels={'Open-Loop'; 'Hybrid-Loop'; 'Closed-Loop'};
%position of labels
XTick=1:1:3;

set(gca, 'XTick',XTick);
set(gca, 'XTickLabel', XTickLabels);

