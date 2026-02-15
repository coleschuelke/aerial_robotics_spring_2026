function [Q] = simulateQuadrotorControl(R,S,P)
% simulateQuadrotorControl : Simulates closed-loop control of a quadrotor
%                            aircraft.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%        rIstar = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
%        vIstar = Nx3 matrix of desired CM velocities with respect to the I
%                 frame and expressed in the I frame, in meters/sec.
%                 vIstar(k,:)' is the 3x1 velocity at time tk = tVec(k).
%
%        aIstar = Nx3 matrix of desired CM accelerations with respect to the I
%                 frame and expressed in the I frame, in meters/sec^2.
%                 aIstar(k,:)' is the 3x1 acceleration at time tk =
%                 tVec(k).
%
%        xIstar = Nx3 matrix of desired body x-axis direction, expressed as a
%                 unit vector in the I frame. xIstar(k,:)' is the 3x1
%                 direction at time tk = tVec(k).
%  
% S ---------- Structure with the following elements:
%
%  oversampFact = Oversampling factor. Let dtIn = R.tVec(2) - R.tVec(1). Then
%                 the output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.
%
%        state0 = State of the quad at R.tVec(1) = 0, expressed as a structure
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
%                 disturbance vector acting on the quad from R.tVec(k) to
%                 R.tVec(k+1).
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% Q ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 Q.tVec(1) = R.tVec(1), Q.tVec(M) = R.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%  
%         state = State of the quad at times in tVec, expressed as a
%                 structure with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the I frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the I frame
%                       and expressed in the I frame, in meters per
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
% Author: Quentin Cole Schuelke
%+==============================================================================+  

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addParameter('tVec',[],@(x)issize(x,NaN,1));
  ip.addParameter('rIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('vIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('aIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('xIstar',[],@(x)issize(x,NaN,3));
  ip.addParameter('oversampFact',[],@(x)issize(x,1,1));
  ip.addParameter('distMat',[],@(x)issize(x,NaN,3));
  ip.addParameter('r',[],@(x)issize(x,3,1));
  ip.addParameter('e',[],@(x)issize(x,3,1));
  ip.addParameter('v',[],@(x)issize(x,3,1));
  ip.addParameter('omegaB',[],@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(R,S,S.state0,P);
  if sum(cellfun(@isempty,struct2cell(ip.Results)))~=0
    warning('Input empty or missing.');
  end
end

%% Student code

% Unpack R

% Unpack S
tVec = R.tVec;
dtIn = tVec(2) - tVec(1);
dtOut = dtIn/S.oversampFact;
N = length(tVec);
x0 = S.state0;
dcm0 = euler2dcm(x0.e);
dist = S.distMat;

% Unpack P
qp = P.quadParams;
c = P.constants;
s = P.sensorParams;

% Create vectorized initial conditions with rotor speed IC's 
x0_aug = [x0.r; x0.v; dcm0(:); x0.omegaB; zeros(4, 1)];

% Initialize the outputs
xOut = [];
tVecOut = [];

% Prep the first initial condition
xk = x0_aug;

% Main loop
for k=1:N-1
    % Normalize R every 100 steps
    if mod(k, 100) == 0
        xk = normalizeR(xk);
    end

    % Create the step
    t_span = [tVec(k):dtOut:tVec(k+1)];

    Sk.statek.rI = xk(1:3);
    Sk.statek.RBI = [xk(7:9), xk(10:12), xk(13:15)];
    Sk.statek.vI = xk(4:6);
    Sk.statek.omegaB = xk(16:18);

    % Create Rt for the trajectory
    Rt.rIstark = R.rIstar(k, :).';
    Rt.vIstark = R.vIstar(k, :).';
    Rt.aIstark = R.aIstar(k, :).';

    % Call the trajectory controller
    [Fk, zIstark] = trajectoryController(Rt, Sk, P);

    % Calculate Ra for the attitude
    Ra.zIstark = zIstark;
    Ra.xIstark = R.xIstar(k, :).';

    % Call the attitude controller
    NBk = attitudeController(Ra, Sk, P);

    % Convert the force and torque to motor voltages
    eak = voltageConverter(Fk, NBk, P);

    % Simulate next step with voltage inputs
    [tOutk, xOutk] = ode45(@(t, x) quadOdeFunctionHF(t, x, eak, dist(k, :).', P), t_span, xk);

    % Save solutions (includes initial condition, excludes final state (initial condition for next step)) 
    tVecOut = [tVecOut; tOutk(1:end-1)];
    xOut = [xOut; xOutk(1:end-1, :)];

    xk = xOutk(end, :).';
end

% Save the last round of solutions, including the very final state
tVecOut = [tVecOut; tOutk(1:end)];
xOut = [xOut; xOutk(1:end, :)];

% Convert DCM back to euler angles
e = [];
for j=1:length(tVecOut)
    e(j, :) = dcm2euler([xOut(j, 7:9).', xOut(j, 10:12).', xOut(j, 13:15).']);
end

% Pack Q output
Q.tVec = tVecOut;
state.rMat = xOut(:, 1:3);
state.vMat = xOut(:, 4:6);
state.eMat = e;
state.omegaBMat = xOut(:, 16:18);

% Ship it!
Q.state = state;

end % EOF simulateQuadrotorControl.m



