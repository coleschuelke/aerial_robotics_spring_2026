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

step(sys)
stepinfo(sys)