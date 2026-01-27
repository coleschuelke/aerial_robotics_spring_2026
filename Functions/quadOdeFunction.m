function [Xdot] = quadOdeFunction(t,X,omegaVec,distVec,P)
% quadOdeFunction : Ordinary differential equation function that models
%                   quadrotor dynamics.  For use with one of Matlab's ODE
%                   solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),omegaB']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%
% omegaVec --- 4x1 vector of rotor angular rates, in rad/sec.  omegaVec(i)
%              is the constant rotor speed setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the quad's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Cole Schuelke
%+==============================================================================+

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addRequired('t',@(x)issize(x,1,1));
  ip.addRequired('X',@(x)issize(x,18,1));
  ip.addRequired('omegaVec',@(x)issize(x,4,1));
  ip.addRequired('distVec',@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(t,X,omegaVec,distVec,P);
end

%% Student code
% Unpack P
qp = P.quadParams;
c = P.constants;

% Definitions for convenience
e1 = [1 0 0].';
e2 = [0 1 0].';
e3 = [0 0 1].';

% Unpack X
rI = X(1:3);
vI = X(13:15);
RBI = [X(4:6), X(7:9), X(10:12)];
omegaB = X(16:18);

% Intermediate Calculations
FiB = (qp.kF*omegVec.^2).'.*e3; % Thrust forces in the body frame
FI = RBI.'*sum(FiB, 2); % Total thrust in the inertial frame
NB_r = sum(((qp.kN*omegaVec.^2).*qp.omegaRdir.').'.*e3, 2);
NB_f = sum(cross(qp.rotor_loc, FiB, 1), 2);
NB = NB_r + NB_f;

% Calculate Xdot
rI_dot = vI;
vI_dot = -qp.m*c.g*e3 + FI + distVec;
RBI_dot = -1*crossProductEquivalent(omegaB)*RBI;
omegaB_dot = qp.Jq\(NB - crossProductEquivalent(omegaB)*qp.Jq*omegaB);

% Repack Xdot
Xdot = [rI_dot; vI_dot; RBI_dot(:); omegaB_dot];

end % EOF quadOdeFunction.m