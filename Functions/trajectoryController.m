function [Fk,zIstark] = trajectoryController(R,S,P)
% trajectoryController : Controls quadcopter toward a reference trajectory.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       rIstark = 3x1 vector of desired CM position at time tk in the I frame,
%                 in meters.
%
%       vIstark = 3x1 vector of desired CM velocity at time tk with respect to
%                 the I frame and expressed in the I frame, in meters/sec.
%
%       aIstark = 3x1 vector of desired CM acceleration at time tk with
%                 respect to the I frame and expressed in the I frame, in
%                 meters/sec^2.
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
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% zIstark ---- Desired 3x1 body z axis expressed in I frame at time tk.    
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
  ip.addParameter('rIstark',[],@(x)issize(x,3,1));
  ip.addParameter('vIstark',[],@(x)issize(x,3,1));
  ip.addParameter('aIstark',[],@(x)issize(x,3,1));
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
rIstark = R.rIstark;
vIstark = R.vIstark;
aIstark = R.aIstark;

% Unpack S
rIk = S.statek.rI;
RBIk = S.statek.RBI;
vIk = S.statek.vI;
omegaBk = S.statek.omegaB;

% Unpack P
qp = P.quadParams;
c = P.consants;

% Definitions for convenience
e1 = [1 0 0].';
e2 = [0 1 0].';
e3 = [0 0 1].';

% Controller gains
k = 0;
kd = 0;

% Position error
erk = rIstark - rIk;
erk_dot = vIstark - vIk;

% Desired thrust 
FIstark = k*erk + kd*erk_dot + qp.m*e3 * qp.m*aIstark;
Fk = FIstark.'*RBIk.'*e3;

% Desired z-axis 
zIstark = FIstark / norm(FIstark);

  
end % EOF trajectoryController.m