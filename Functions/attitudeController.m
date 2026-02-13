function [NBk] = attitudeController(R,S,P)
% attitudeController : Controls quadcopter toward a reference attitude
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       zIstark = 3x1 desired body z-axis direction at time tk, expressed as a
%                 unit vector in the I frame.
%
%       xIstark = 3x1 desired body x-axis direction, expressed as a
%                 unit vector in the I frame.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
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
  ip.addParameter('zIstark',[],@(x)issize(x,3,1));
  ip.addParameter('xIstark',[],@(x)issize(x,3,1));
  ip.addParameter('rI',[],@(x)issize(x,3,1));
  ip.addParameter('RBI',[],@(x)issize(x,3,3));
  ip.addParameter('vI',[],@(x)issize(x,3,1));
  ip.addParameter('omegaB',[],@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(R,S.statek,P);
end

%% Student code
% Unpack R
zIstark = R.zIstark;
xIstark = R.xIstark;

% Unpack S
rIk = S.statek.rI;
RBIk = S.statek.RBI;
vIk = S.statek.vI;
omegaBk = S.statek.omegaB;

% Unpack P
qp = P.quadParams;
c = P.constants;

% Definitions for convenience
e1 = [1 0 0].';
e2 = [0 1 0].';
e3 = [0 0 1].';

% Controller gains
K = diag([0.3 0.5 1]); % [kx, ky, kz]
Kd = diag([.1 .1 .1]); % [kdx, kdy, kdz]

% RBIstark
zxxk = cross(zIstark, xIstark);
b = zxxk / norm(zxxk);
a = cross(b, zIstark);
RBIstark = [a, b, zIstark].';

% Error DCM
REk = RBIstark*RBIk.';

% Extract the eigenaxis of rotation
eEk = [REk(2, 3) - REk(3, 2); REk(3, 1) - REk(1, 3); REk(1, 2) - REk(2, 1)];

% Control Law
NBk = K*eEk - Kd*omegaBk + crossProductEquivalent(omegaBk)*qp.Jq*omegaBk;
  
end % EOF attitudeController.m