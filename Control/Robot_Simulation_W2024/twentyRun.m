openLoopScore = zeros(20,1);
hybridLoopScore = zeros(20,1);
closedLoopScore = zeros(20,1);

%runs the simulation 20 times and appends each final score into an array.
for k = 1:20
     robot_simulation;
     openLoopScore(k) = sim.score(end);
end