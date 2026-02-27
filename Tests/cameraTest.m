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

S.statek.RBI = sensorParams.RCB.'; % Align Frames
S.statek.rI = -S.statek.RBI.'*sensorParams.rocB; % Make origins coincident

rXI = [0, 0, 0].';

rx = hdCameraSimulator(rXI, S, P)