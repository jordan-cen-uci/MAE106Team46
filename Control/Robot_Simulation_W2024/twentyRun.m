openLoopScore = zeros(20,1);
hybridLoopScore = zeros(20,1);
closedLoopScore = zeros(20,1);


for k = 1:20
     robot_simulation;
     openLoopScore(k) = sim.score(end);
end