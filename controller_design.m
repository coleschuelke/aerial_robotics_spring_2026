clear variables;
close all;
clc;

k = 1;
kd = 0.5;

% Create the closed loop TF 
numerator = [kd, k];
denom = [1, kd, k];

sys = tf(numerator, denom);

step(sys)
stepinfo(sys)