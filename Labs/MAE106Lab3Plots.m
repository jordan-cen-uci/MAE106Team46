part3 = readtable("Lab3Part3.csv");

xInput = linspace(1, 5, 234);

solenoid1 = part3.Solenoid1;
solenoid2 = part3.Solenoid2;
magnet1 = part3.Magentometer1;
magnet2 = part3.Magnetometer2;
filterStrength = 0.95;
filteredMagnet1 = magnet1(1);
filteredMagnet2 = magnet2(1);

for k = 2:length(magnet1)
    filteredMagnet1 = [filteredMagnet1; (1-filterStrength)*magnet1(k) + filterStrength*filteredMagnet1(k-1)];
end

for k = 2:length(magnet2)
    filteredMagnet2 = [filteredMagnet2; (1-filterStrength)*magnet2(k) + filterStrength*filteredMagnet2(k-1)];
end

figure(1);
hold on;
plot(xInput, magnet1,'r-');
plot(xInput, filteredMagnet1, 'b-');
plot(xInput, magnet2, "Color", [0.9290 0.6940 0.1250]);
plot(xInput, filteredMagnet2, '-', "Color", "green");
title("Comparing Effects of Distance to Solenoid and Filteration on Magnotometer Reading");
legend('Close to Solenoid; Unfiltered', 'Close to Solenoid; Filtered \gamma = 0.95', 'Far from Solenoid; Unfiltered', 'Far from Solenoid; Filtered \gamma = 0.95', 'Location','southeast');
xlabel('Time (s)');
ylabel('Heading of Magnotometer (Degrees)');
