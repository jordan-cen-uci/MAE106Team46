%part 1

kp1Time =
kp1DesVel =
kp1Vel =
kp1U = -1.*(kp1Vel - kp1DesVel);
kp1ESS = kp1Vel(end) - kp1DesVel(end);

kp2Time =
kp2DesVel =
kp2Vel =
kp2U = -2.*(kp2Vel - kp2DesVel);
kp2ESS = kp2Vel(end) - kp2DesVel(end);

kp3Time =
kp3DesVel =
kp3Vel =
kp3U = -3.*(kp3Vel - kp3DesVel);
kp3ESS = kp3Vel(end) - kp3DesVel(end);

kp4Time =
kp4DesVel =
kp4Vel =
kp1U = -4.*(kp4Vel - kp4DesVel);
kp1ESS = kp4Vel(end) - kp4DesVel(end);

ESS = [kp1ESS; kp2ESS; kp3ESS, kp4ESS];

%figure 1, all velocities in terms of time with desired velocity, show this
%to TA
figure(1);
hold on;
plot(kp1, kp1DesVel); %plots the desired velocity, should be same at 60 rpm
plot(kp1Time, kp1Vel); %plots velocity with Kp of 1
plot(kp2Time, kp2Vel);
plot(kp3Time, kp3Vel);
plot(kp4Time, kp4Vel);

%figure 2, plots the input function of U for each K value
figure(2);
hold on;
plot(kp1Time, kp1U);
plot(kp2Time, kp2U);
plot(kp3Time, kp3U);
plot(kp4Time, kp4U);

%fgiure 3, plots the error steady state (desired - actual) in terms of time
figure(3);
hold on;
plot([1; 2; 3; 4], ESS);

figure(4);
hold on;
plot([1; 2; 3; 4], )

%part 2

f1Time = 
f1DesVel = 
f1Vel = 

f5Time = 
f5DesVel = 
f5Vel = 

f10Time = 
f10DesVel = 
f10Vel = 

f20Time = 
f20DesVel = 
f20Vel = 

f40Time = 
f40DesVel = 
f40Vel = 

figure(5);
hold on;
subplot(3, 2, 1);
plot(f1Time, f1Vel);

subplot(3, 2, 2);
plot(f5Time, f5Vel);

subplot(3, 2, 3);
plot(f10Time, f10Vel);

subplot(3, 2, 4);
plot(f20Time, f20Vel);

subplot(3, 2, 5);
plot(f40Time, f40Vel);

