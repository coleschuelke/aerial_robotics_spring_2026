clear variables;
close all;
clc;

addpath("../Params")
addpath("../Functions/")

sensorParamsScript;
P.sensorParams = sensorParams;

S.statek.rI = [4;12;3];
S.statek.RBI = eye(3);

[rpGtilde, rbGtilde] = gnssMeasSimulator(S, P);

RLG = Recef2enu(sensorParams.r0G);

rIout = RLG*rpGtilde