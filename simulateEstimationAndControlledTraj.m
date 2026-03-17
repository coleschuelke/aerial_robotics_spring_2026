% Top-level script for calling simulateQuadrotorDynamics
clear all; clc;
close all;
global INPUT_PARSING;
INPUT_PARSING = false;
addpath("Functions/");
addpath("Params/");
addpath("Data/");
addpath("Analysis/");
% rng seed for debugging
rng(52);

% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
sensorParamsScript;
Pin.quadParams = quadParams;
Pin.constants = constants;
Pin.sensorParams = sensorParams;


% Import trajectory
r_planned = readmatrix("position.csv");
v_planned = readmatrix("velocity.csv");
a_planned = readmatrix("acceleration.csv");

% Time vector, in seconds
t = r_planned(1, :);
% Repackage trajectories
xI = r_planned(2, :);
yI = r_planned(3, :);
xI_dot = v_planned(2, :);
yI_dot = v_planned(3, :);
xI_ddot = a_planned(2, :);
yI_ddot = a_planned(3, :);
z = zeros(1, length(xI));
% Desired attitude (along trajectory)
atti = xI_dot;
attj = yI_dot;

% Pack R
R.tVec = t;
R.rIstar = [ xI.', yI.', z.'];
R.vIstar = [xI_dot.', yI_dot.', z.'];
R.aIstar = [xI_ddot.', yI_ddot.', z.'];
R.xIstar = [atti.', attj.', z.'];
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = 3*randn(length(t), 3); % With disturbances
% S.distMat = zeros(length(t), 3); % No disturbances
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
% Random Features
S.rXIMat = unifrnd(-10, 10, 10, 3);

% load("Data/Stest");
[P, Est] = simulateQuadrotorEstimationAndControl(R, S, Pin);

%% Animation
S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=1*[-1 10 -10 1 -5 5];
visualizeQuad(S2);

%% Plotting
plotQuad(R, P, Est)
