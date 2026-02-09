function [P] = simulateQuadrotorDynamicsHF(S)
% simulateQuadrotorDynamicsHF : Simulates the dynamics of a quadrotor
%                               aircraft (high-fidelity version).
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%  oversampFact = Oversampling factor. Let dtIn = tVec(2) - tVec(1). Then the
%                 output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.   
%
%         eaMat = (N-1)x4 matrix of motor voltage inputs.  eaMat(k,j) is the
%                 constant (zero-order-hold) voltage for the jth motor over
%                 the interval from tVec(k) to tVec(k+1).
%
%        state0 = State of the quad at tVec(1) = 0, expressed as a structure
%                 with the following elements:
%                   
%                   r = 3x1 position in the world frame, in meters
% 
%                   e = 3x1 vector of Euler angles, in radians, indicating the
%                       attitude
%
%                   v = 3x1 velocity with respect to the world frame and
%                       expressed in the world frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%       distMat = (N-1)x3 matrix of disturbance forces acting on the quad's
%                 center of mass, expressed in Newtons in the world frame.
%                 distMat(k,:)' is the constant (zero-order-hold) 3x1
%                 disturbance vector acting on the quad from tVec(k) to
%                 tVec(k+1).
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
% P ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 P.tVec(1) = S.tVec(1), P.tVec(M) = S.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%                  
%  
%         state = State of the quad at times in tVec, expressed as a structure
%                 with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the world frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the world frame
%                       and expressed in the world frame, in meters per
%                       second.
%                 
%           omegaBMat = Mx3 matrix composed such that omegaBMat(k,:)' is the
%                       3x1 angular rate vector expressed in the body frame in
%                       radians, that applies at tVec(k).
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addParameter('tVec',[],@(x)issize(x,NaN,1));
  ip.addParameter('oversampFact',[],@(x)issize(x,1,1));
  ip.addParameter('eaMat',[],@(x)issize(x,NaN,4));
  ip.addParameter('distMat',[],@(x)issize(x,NaN,3));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.addParameter('r',[],@(x)issize(x,3,1));
  ip.addParameter('e',[],@(x)issize(x,3,1));
  ip.addParameter('v',[],@(x)issize(x,3,1));
  ip.addParameter('omegaB',[],@(x)issize(x,3,1));
  ip.parse(S,S.state0);
  if sum(cellfun(@isempty,struct2cell(ip.Results)))~=0
    warning('Input empty or missing.');
  end
end

%% Student code

% Unpack S
tVec = S.tVec;
dtIn = tVec(2) - tVec(1);
dtOut = dtIn/S.oversampFact;
N = length(tVec);
x0 = S.state0;
dcm0 = euler2dcm(x0.e);
eaMat = S.eaMat;
dist = S.distMat;

% Create vectorized x0 and add rotor speed initial conditions
x0_aug = [x0.r; x0.v; dcm0(:); x0.omegaB; zeros(4, 1)];

% Repackage the params
Params.quadParams = S.quadParams;
Params.constants = S.constants;

% Initialize outputs 
xOut = [];
tVecOut = [];

% Prep first initial condition
xi = x0_aug;

% Propagate
for i=1:N-1
    t_span = [tVec(i):dtOut:tVec(i+1)];
    % Should normalize C here with some regularity.

    [tVeci, xOuti] = ode45(@(t,x) quadOdeFunctionHF(t, x, eaMat(i, :).', dist(i, :).', Params), t_span, xi);
    
    % Save solutions (includes initial condition, excludes final state (initial condition for next step)) 
    tVecOut = [tVecOut; tVeci(1:end-1)];
    xOut = [xOut; xOuti(1:end-1, :)];

    xi = xOuti(end, :)';
end

% Save the last round of solutions, including the very final state
tVecOut = [tVecOut; tVeci(1:end)];
xOut = [xOut; xOuti(1:end, :)];

% Convert DCM back to euler angles
e = [];
for j=1:length(tVecOut)
    e(j, :) = dcm2euler([xOut(j, 7:9).', xOut(j, 10:12).', xOut(j, 13:15).']);
end

% Save the last round of solutions, including the very final state
tVecOut = [tVecOut; tVeci(1:end)];
xOut = [xOut; xOuti(1:end, :)];

% Repack P
P.tVec = tVecOut;
state.rMat = xOut(:, 1:3);
state.vMat = xOut(:, 4:6);
state.eMat = e;
state.omegaBMat = xOut(:, 16:18);

% Ship it!
P.state = state;

end % EOF simulateQuadrotorDynamicsHF.m