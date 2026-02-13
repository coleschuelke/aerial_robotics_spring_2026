clear variables;
close all;
clc;

%% Position (same in all dimensions)
run("../Params/quadParamsScript.m")
qp = quadParams;

km = 1/qp.m;

massPlant = tf(km, [1 0 0]);

k = 37.5;
kd = 6.8;

CLnum = [km*kd, km*k];
CLdenom = [1, km*kd, km*k];

massCLTF = tf(CLnum, CLdenom);

%% Attitude (Different per axis)
kx = 1/qp.Jq(1, 1);
ky = 1/qp.Jq(2, 2);
kz = 1/qp.Jq(3, 3);

jxPlant = tf(kx, [1 0 0]);
jyPlant = tf(ky, [1 0 0]);
jzPlant = tf(kz, [1 0 0]);

kpx = 0.09412;
kdx = 0.01483;

kpy = 0.21;
kdy = 0.0289;

kpz = 0.271;
kdz = 0.03856;

jxCLTF = tf([kx*kdx, kx*kpx], [1, kx*kdx, kx*kpx]);
jyCLTF = tf([ky*kdy, ky*kpy], [1, ky*kdy, ky*kpy]);
jzCLTF = tf([kz*kdz, kz*kpz], [1, kz*kdz, kz*kpz]);