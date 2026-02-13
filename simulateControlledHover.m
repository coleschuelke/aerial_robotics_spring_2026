% Top-level script for calling simulateQuadrotorDynamics
clear; clc;
close all;
global INPUT_PARSING;
INPUT_PARSING = false;
addpath("Functions/");
addpath("Params/");
addpath("Data/")
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
Pin.quadParams = quadParams;
Pin.constants = constants;
Pin.sensorParams = 0;
% Total simulation time, in seconds
Tsim = 7;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;

% Setup
T = Tsim;
r = 4;
t = 0:delt:T;

% Desired attitude 
xi = -sin((2*pi/T)*t);
xj = -cos((2*pi/T)*t);
xk = zeros(1, length(t));

% Time vector, in seconds 
N = floor(Tsim/delt);
R.tVec = [0:N-1]'*delt;
R.rIstar = zeros([N, 3]);
R.vIstar = zeros([N, 3]);
R.aIstar = zeros([N, 3]);
% R.xIstar = [ones([N, 1]), zeros([N, 2])];
R.xIstar = [xi.', xj.', xk.']; 
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = 0.5*randn((N-1), 3);
% Rotor speeds at each time, in rad/s
S.eaMat = 586/quadParams.cm(1)*ones(N-1,4);
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

% load("Data/Stest");
P = simulateQuadrotorControl(R, S, Pin);

S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=1*[-10 10 -10 10 -10 10];
visualizeQuad(S2);

figure(1);clf;
plot(P.tVec,P.state.rMat(:,3)); grid on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

figure(2);clf;
plot(P.state.rMat(:,1), P.state.rMat(:,2)); 
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');