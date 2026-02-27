function [ftildeB,omegaBtilde] = imuSimulator(S,P)
% imuSimulator : Simulates IMU measurements of specific force and
%                body-referenced angular rate.
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
%
%                  vI = 3x1 velocity of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%
%                  aI = 3x1 acceleration of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second^2.
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%           omegaBdot = 3x1 time derivative of omegaB, in radians per
%                       second^2.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% ftildeB ---- 3x1 specific force measured by the IMU's 3-axis accelerometer
%
% omegaBtilde  3x1 angular rate measured by the IMU's 3-axis rate gyro
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Quentin Cole Schuelke
%+==============================================================================+

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addParameter('rI',[],@(x)issize(x,3,1))
  ip.addParameter('vI',[],@(x)issize(x,3,1))
  ip.addParameter('aI',[],@(x)issize(x,3,1))
  ip.addParameter('RBI',[],@(x)issize(x,3,3))
  ip.addParameter('omegaB',[],@(x)issize(x,3,1))
  ip.addParameter('omegaBdot',[],@(x)issize(x,3,1))
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(S.statek,P);
end

%% Student code
persistent bakm1 bgkm1

% Unpack S
RBI = S.statek.RBI;
aI = S.statek.aI;
omegaB = S.statek.omegaB;
omegaBdot = S.statek.omegaBdot;

% Unpack P
sp = P.sensorParams;
c = P.constants;

% Convenient definitions
e3 = [0, 0, 1].';

% Handle bias from last step
if(isempty(bakm1))
    qbass = sp.Qa2 / (1 - sp.alphaa^2);
    bakm1 = mvnrnd(zeros(3, 1), qbass).'; % Initialize with a ss value
end
if isempty(bgkm1)
    qbgss = sp.Qg2 / (1 - sp.alphag^2);
    bgkm1 = mvnrnd(zeros(3, 1), qbgss).'; % Inialize with a ss value
end

% Calculate current accelerometer bias
bak = sp.alphaa * bakm1 + mvnrnd(zeros(3, 1), sp.Qa2).';

% Calculate specific force measurement
% ftildeB = RBI*(aI + c.g*e3) + cross(omegaB, cross(omegaB, sp.lB)) + cross(omegaBdot, sp.lB) + bak + mvnrnd(zeros(3, 1), sp.Qa).';
ftildeB = RBI*(aI + c.g*e3) + bak + mvnrnd(zeros(3, 1), sp.Qa).';

% Calculate current gyro bias
bgk = sp.alphag * bgkm1 + mvnrnd(zeros(3, 1), sp.Qg2).';

% Calculate rate measurement
omegaBtilde = omegaB + bgk + mvnrnd(zeros(3, 1), sp.Qg).';

% Update the biases for next step
bakm1 = bak;
bgkm1 = bgk;

end % EOF imuSimulator.m