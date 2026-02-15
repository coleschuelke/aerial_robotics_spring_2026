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
Tsim = 10;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);

% Math for trajectory
T = 10;
r = 4;
t = 0:delt:T;
% xI trajectory 
xI = r * sin((2*pi/T)*t);
xI_dot = 2*r*pi/T * cos((2*pi/T)*t);
xI_ddot = -4*r*pi^2/T^2 * sin((2*pi/T)*t);
% yI trajectory
yI = -r * cos((2*pi/T)*t);
yI_dot = 2*r*pi/T * sin((2*pi/T)*t);
yI_ddot = 4*r*pi^2/T^2 * cos((2*pi/T)*t);
% zI trajectory
zI = zeros(1, length(t));
zI_dot = zeros(1, length(t));
zI_ddot = zeros(1, length(t));
% Desired attitude 
xi = -sin((2*pi/T)*t);
xj = cos((2*pi/T)*t);
xk = zeros(1, length(t));


% Padding to let altitude settle
xSettle = zeros(N-length(t), 1);
ySettle = -4*ones(N-length(t), 1);
zSettle = zeros(N-length(t), 1);
xjSettle = ones(N-length(t), 1);

% Pack R
R.tVec = [0:N]'*delt;
R.rIstar = [xSettle, ySettle, zSettle; xI.', yI.', zI.'];
R.vIstar = [zeros(N-length(t), 3); xI_dot.', yI_dot.', zI_dot.'];
R.aIstar = [zeros(N-length(t), 3); xI_ddot.', yI_ddot.', zI_ddot.'];
R.xIstar = [xSettle, xjSettle, zSettle; xi.', xj.', xk.'];
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
% S.distMat = 0.5*randn((N-1), 3); % With disturbances
S.distMat = zeros(N, 3); % No disturbances
% Initial position in m
S.state0.r = [0 -r 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 pi/2]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
% Oversampling factor
S.oversampFact = 10;

% load("Data/Stest");
P = simulateQuadrotorControl(R, S, Pin);

% Calculations for plotting xI
xI_true = zeros(2, length(P.state.eMat));
for j=1:length(xI_true)
    xtemp = euler2dcm(P.state.eMat(j, :)).'*[1 0 0].';
    xI_true(:, j) = xtemp(1:2);
end

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
plot(P.tVec,P.state.rMat(:,3), 'b-', 'LineWidth', 3); grid on;
yline(0, 'k-.');
xlabel('Time (sec)', 'FontSize', 20);
ylabel('Vertical (m)', 'FontSize', 20);
set(gca, 'FontSize', 14);
title('Vertical position of CM', 'FontSize', 20);
legend('Flown Height', 'Planned Height', 'FontSize', 12)

figure(2);clf;
hold on;
plot(xI, yI, 'k-.', 'LineWidth', 3);
plot(P.state.rMat(:,1), P.state.rMat(:,2), 'b-', 'LineWidth', 3); 
plot(0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 5);
for i=1:length(xI_true)
    if mod(i, 500) == 0
        plot([P.state.rMat(i, 1), P.state.rMat(i, 1)+xI_true(1, i)], [P.state.rMat(i, 2), P.state.rMat(i, 2)+xI_true(2, i)], 'r-', 'LineWidth', 2);
    end
end
axis equal; grid on;
xlabel('X (m)', 'FontSize', 20);
ylabel('Y (m)', 'FontSize', 20);
set(gca, 'FontSize', 14);
title('Horizontal position of CM', 'FontSize', 20);
legend('Planned Trajectory', 'Flown Trajectory', 'Circle Center', 'x_b Direction', 'FontSize', 12);