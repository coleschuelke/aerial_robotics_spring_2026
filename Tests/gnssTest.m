clear variables;
close all;
clc;

addpath("../Params")
addpath("../Functions/")

sensorParamsScript;
quadParamsScript;
constantsScript;
P.sensorParams = sensorParams;
P.constants = constants;
P.quadParams = quadParams;

% Test the measurement simulator
S.statek.rI = [0;0;0];
S.statek.RBI = eye(3);

[rpGtilde, rbGtilde] = gnssMeasSimulator(S, P);

RLG = Recef2enu(sensorParams.r0G);

rIout = RLG*rpGtilde
rbout = RLG*rbGtilde

% Test the measurement model
xk = zeros(15, 1);
wk = zeros(6, 1);
RBIBark = eye(3);
rXIMat = [];
mcVec = [];

zk = h_meas(xk, wk, RBIBark, rXIMat, mcVec, P);