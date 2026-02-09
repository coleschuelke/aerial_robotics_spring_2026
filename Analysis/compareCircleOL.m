% Top-level script for calling simulateQuadrotorDynamics
clear; clc;
close all;
global INPUT_PARSING;
INPUT_PARSING = false;
addpath("Functions/");
addpath("Params/");
% Total simulation time, in seconds
Tsim = 4;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
S.tVec = [0:N-1]'*delt;
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = zeros(N-1,3);
% Rotor speeds at each time, in rad/s
S.omegaMat = 585*ones(N-1,4);
% Initial position in m
S.state0.r = [0 0 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0.05 0.2]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [-0.8 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0.8]';
% Oversampling factor
S.oversampFact = 10;
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
S.quadParams = quadParams;
S.constants = constants;

load Stest; % Loads S
Ptest = load("Ptest.mat"); % Loads Ptest
Ptest = Ptest.P;

P = simulateQuadrotorDynamics(S);

S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=1*[-1 1 -1 1 -1 1];
visualizeQuad(S2);

S3.tVec = Ptest.tVec;
S3.rMat = Ptest.state.rMat;
S3.eMat = Ptest.state.eMat;
S3.plotFrequency = 20;
S3.makeGifFlag = false;
S3.gifFileName = 'testGif.gif';
S3.bounds=1*[-1 1 -1 1 -1 1];
visualizeQuad(S3);

figure(1);clf;
plot(P.tVec,P.state.rMat(:,3), "LineWidth",5); grid on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

figure(2);clf;
plot(P.state.rMat(:,1), P.state.rMat(:,2), "LineWidth",5); 
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');

figure(3);clf;
plot(Ptest.tVec,Ptest.state.rMat(:,3), "LineWidth",5); grid on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

figure(4);clf;
plot(Ptest.state.rMat(:,1), Ptest.state.rMat(:,2), "LineWidth",5); 
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');