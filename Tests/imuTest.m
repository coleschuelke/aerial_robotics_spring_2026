clear variables;
close all;
clc;

addpath("../Params");
addpath("../Functions");

sensorParamsScript;
constantsScript;

P.sensorParams = sensorParams;
P.constants = constants;

statek.rI = zeros(3, 1);
statek.vI = zeros(3, 1);
statek.aI = zeros(3, 1);
statek.RBI = eye(3);
statek.omegaB = [0 0 0].';
statek.omegaBdot = zeros(3, 1);
S.statek = statek;

[ftildeB, omegaBtilde] = imuSimulator(S, P)