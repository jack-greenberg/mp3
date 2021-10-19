M = csvread('run.log');

close all;

figure;

title("Run data");

s = length(M(:,1));
xlim([0 s]);

subplot(2, 1, 1);
hold on; grid on; legend;
xlabel("Time");
ylabel("Voltage");

plot(M(:,1), 'r', 'DisplayName', 'Sensor');
hold off;

subplot(2, 1, 2);
hold on; grid on; legend;
xlabel("Time");
ylabel("Motor speed");

plot(M(:,2), 'g', 'DisplayName', 'Left motor speed');
plot(M(:,3), 'b', 'DisplayName', 'Right motor speed');
hold off;
