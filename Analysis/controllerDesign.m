clear variables;
close all;
clc;

k = 45;
kd = 7;

% Create the closed loop TF 
numerator = [kd, k];
denom = [1, kd, k];

sys = tf(numerator, denom);

plant = tf(1, [1 0 0]);

stepinfo(sys)
[y, t] = step(sys);
figure;
hold on;
plot(t, y, 'b-', 'LineWidth', 3);
set(gca, 'FontSize', 14);
yline(0.9, 'm-.', 'LineWidth', 3);
yline([1.02, 0.98], 'g-.', 'LineWidth', 3);
yline(1.2865, 'r-.', 'LineWidth', 3);
xline(1.11, 'g-.', 'LineWidth', 3);
xline(0.154, 'm-.', 'LineWidth', 3);
text(0.3, 0.87, '90% Step', 'Color', 'magenta', 'FontSize', 20);
text(1.2, 1.05, '2% Bounds', 'Color', 'green', 'FontSize', 20);
text(0.7, 1.25, '28.6% Overshoot', 'Color', 'red', 'FontSize', 20);
text(0.18, 0.65, 'Rise Time = 0.15s', 'Color', 'magenta', 'FontSize', 20, 'Rotation', -90);
text(1.14, 0.65, 'Settling Time = 1.1s', 'Color', 'green', 'FontSize', 20, 'Rotation', -90);
xlabel('Time (s)', 'FontSize', 20);
ylabel('Amplitude', 'FontSize',20);
title('Step Response', 'FontSize', 20);