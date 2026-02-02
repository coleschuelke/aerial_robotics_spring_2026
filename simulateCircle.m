% Top-level script for calling simulateQuadrotorDynamics
clear; clc;
close all;
global INPUT_PARSING;
INPUT_PARSING = false;
addpath("Functions/");
addpath("Params/");
% Total simulation time, in seconds
T = 4.525;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(T/delt);
S.tVec = [0:N-1]'*delt;

% Definitions for convenience
e1 = [1, 0, 0].';
e2 = [0, 1, 0].';
e3 = [0, 0, 1].';
rp = 0.21; % m

% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
S.quadParams = quadParams;
S.constants = constants;
qp = quadParams;
const = constants;

% Calculations
r = 0.75; % m
c = 2*pi*r; % m
vt = c/T; % m/s
ac = (vt^2)/r; % m/s2
Fc = qp.m*ac; % N
Fl = qp.m*const.g; % N
theta = atan2(ac, const.g); % rad
phi = 0; % rad
psi0 = 0; % rad
psi_dot = 2*pi/T; % rad/s
Ft = Fc / sin(theta); % N

omegaB = psi_dot * rotationMatrix(e2, theta) * rotationMatrix(e1, phi) * e3; % rad/s (I think units are right)
NB = crossProductEquivalent(omegaB) * qp.Jq * omegaB; % If coordinate system was not centered at the CM, then we would have to compute Jcm from parallel axis theorem

Fo = (NB(2) + Ft*rp) / (2*rp);
Fi = Ft - Fo;
omegai = ((Fi*0.5) / (qp.kF(1)))^0.5;
omegao = ((Fo*0.5) / (qp.kF(1)))^0.5;

disp("Theta: ")
disp(theta)

disp("vertical thrust: ")
disp(Fi+Fo)
disp(Ft)


% INTITIAL CONDITIONS
% Initial position in m
S.state0.r = [0 -r 0.5]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 theta pi/2]'; % Make sure this is expected orientation
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [vt 0 0]'; % Need to make sure this is in the proper direction according to omegaB
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = omegaB; % This should be directly transferable 

% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = zeros(N-1,3);
% Rotor speeds at each time, in rad/s
S.omegaMat = [omegai, omegai, omegao, omegao].*ones(N-1,4); % No idea if this is correct
% Oversampling factor
S.oversampFact = 10;

P = simulateQuadrotorDynamics(S);

S2.tVec = P.tVec;
S2.rMat = P.state.rMat;
S2.eMat = P.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds=1*[-1 1 -1 1 -1 1];
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