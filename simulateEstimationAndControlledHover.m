% Top-level script for calling simulateQuadrotorDynamics
clear all; clc;
close all;
global INPUT_PARSING;
INPUT_PARSING = false;
addpath("Functions/");
addpath("Params/");
addpath("Data/")
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
sensorParamsScript;
Pin.quadParams = quadParams;
Pin.constants = constants;
Pin.sensorParams = sensorParams;
% Total simulation time, in seconds
Tsim = 15;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;

% Setup
T = Tsim;
r = 4;
t = 0:delt:T;

% Desired attitude 
xi = -sin((2*pi/T)*2*t);
xj = cos((2*pi/T)*2*t);
xk = zeros(1, length(t));

% Time vector, in seconds 
N = floor(Tsim/delt);
R.tVec = [0:N]'*delt;
R.rIstar = zeros([N, 3]);
R.vIstar = zeros([N, 3]);
R.aIstar = zeros([N, 3]);
% R.xIstar = [ones([N, 1]), zeros([N, 2])];
R.xIstar = [xi.', xj.', xk.']; 
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = 0.05*randn(N, 3);
% S.distMat = zeros(N, 3);
% Initial position in m
S.state0.r = [0 0 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 pi/2]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
% Oversampling factor
S.oversampFact = 10;
% Random Features
S.rXIMat = unifrnd(-10, 10, 10, 3);

[P, Est] = simulateQuadrotorEstimationAndControl(R, S, Pin);

%% Animation
S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=1*[-5 5 -5 5 -5 5];
visualizeQuad(S2);

%% Plotting
figure(1);clf;
plot(P.tVec,P.state.rMat(:,3), 'b-', 'LineWidth', 3); grid on; hold on;
plot(Est.tVec, Est.state.rMat(:, 3), 'y.', 'LineWidth', 3);
yline(0, 'k-.');
xlabel('Time (sec)', 'FontSize', 20);
ylabel('Vertical (m)', 'FontSize', 20);
set(gca, 'FontSize', 14);
title('Vertical position of CM', 'FontSize', 20);
legend('Flown Height', 'Planned Height', 'FontSize', 12)

figure(2);clf;
hold on;
plot(zeros(N, 3), zeros(N, 3), 'k-.', 'LineWidth', 3);
plot(P.state.rMat(:,1), P.state.rMat(:,2), 'b-', 'LineWidth', 3);
plot(Est.state.rMat(:, 1), Est.state.rMat(:, 2), 'y.', 'LineWidth', 3);
plot(0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 5);
axis equal; grid on;
xlabel('X (m)', 'FontSize', 20);
ylabel('Y (m)', 'FontSize', 20);
set(gca, 'FontSize', 14);
title('Horizontal position of CM', 'FontSize', 20);
legend('Planned Trajectory', 'Flown Trajectory', 'Estimated Trajectory', 'Circle Center', 'x_b Direction', 'FontSize', 12);

figure(3);clf; hold on;
plot(P.tVec,P.state.eMat(:,1), 'r-', 'LineWidth', 3);
plot(P.tVec,P.state.eMat(:,2), 'g-', 'LineWidth', 3);
plot(P.tVec,P.state.eMat(:,3), 'b-', 'LineWidth', 3); 
plot(Est.tVec,Est.state.eMat(:, 1), 'r.', 'LineWidth', 3);
plot(Est.tVec,Est.state.eMat(:, 2), 'g.', 'LineWidth', 3);
plot(Est.tVec,Est.state.eMat(:, 3), 'b.', 'LineWidth', 3);grid on;
yline(0, 'k-.');
xlabel('Time (sec)', 'FontSize', 20);
ylabel('Angle (rad)', 'FontSize', 20);
set(gca, 'FontSize', 14);
title('Euler Angles', 'FontSize', 20);
legend('Phi', 'Theta', 'Psi', 'FontSize', 12)