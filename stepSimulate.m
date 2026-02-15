% Top-level script for calling simulateQuadrotorDynamics
clear; clc;
close all;
global INPUT_PARSING;
INPUT_PARSING = false;
addpath("Functions/");
addpath("Params/");
% Total simulation time, in seconds
Tsim = 5;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
S.tVec = [0:N-1]'*delt;
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = zeros(N-1,3);
% Rotor speeds at each time, in rad/s
S.omegaMat = [585.5*ones(1, 4); 600*ones(N-2,4)];
% Initial position in m
S.state0.r = [0 0 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 0]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
% Oversampling factor
S.oversampFact = 10;
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
S.quadParams = quadParams;
S.constants = constants;

% Make S2
S2 = S;
S2.omegaMat = [585.5*ones(1, 4); 800*ones(N-2,4)];

% Make S3
S3 = S;
S3.omegaMat = [585.5*ones(1, 4); 1000*ones(N-2,4)];

P = simulateQuadrotorDynamics(S);
P2 = simulateQuadrotorDynamics(S2);
P3 = simulateQuadrotorDynamics(S3);

figure(1);clf;
hold on;
plot(P.tVec,P.state.rMat(:,3), 'LineWidth', 3); 
plot(P2.tVec,P2.state.rMat(:,3), 'LineWidth', 3);
plot(P3.tVec,P3.state.rMat(:,3), 'LineWidth', 3);grid on;
set(gca, 'FontSize', 14);
xlabel('Time (sec)', 'FontSize', 20);
ylabel('Vertical (m)', 'FontSize', 20);
ylim([0, 250]);
title('Vertical position of CM', 'FontSize', 20); 
legend('\omega = 600', '\omega = 800', '\omega = 1000', 'FontSize', 12)