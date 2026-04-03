clear variables; 
close all;
clc;

addpath("..\");
addpath('..\Params\');

sensorParamsScript;
P.sensorParams = sensorParams;


N  = 100; % Number of measurements to take
rXI = [0;0;0.5]; % True location of feature point

estVec = zeros(3, N);
pVec = zeros(3, 3, N);

for m=1:N
    % Set state of quad at time first image is taken
    S.statek.rI = [1;0;1];
    S.statek.RBI = euler2dcm([0;0;pi]);
    
    % Simulate first image and populate measurement arrays
    [rx] = hdCameraSimulator(rXI,S,P);
    M.rxArray{1} = rx;
    M.RCIArray{1} = P.sensorParams.RCB * S.statek.RBI;
    M.rcArray{1} = S.statek.rI + S.statek.RBI'*P.sensorParams.rocB;
    
    % Set state of quad at time second image is taken
    S.statek.rI = [0.7071;0.7071;1];
    S.statek.RBI = euler2dcm([0;0;5*pi/4]);
    
    % Simulate second image and populate measurement arrays
    [rx] = hdCameraSimulator(rXI,S,P);
    M.rxArray{2} = rx;
    M.RCIArray{2} = P.sensorParams.RCB * S.statek.RBI;
    M.rcArray{2} = S.statek.rI + S.statek.RBI'*P.sensorParams.rocB;
    
    % Estimate 3D feature location
    [rXIHat,Px] = estimate3dFeatureLocation(M,P);
    
    % Save the estimate
    estVec(:, m) = rXIHat;
    pVec(:, :, m) = Px;
end

fprintf("The mean error is ");
meanError = mean(abs(estVec-rXI), 2)

fprintf("The mean variance is ");
meanCov = sqrt(diag(mean(pVec, 3))) % Think about whether this is a theoretically meaningful statistic??
