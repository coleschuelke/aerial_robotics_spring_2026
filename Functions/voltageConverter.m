function [eak] = voltageConverter(Fk,NBk,P)
% voltageConverter : Generates output voltages appropriate for desired
%                    torque and thrust.
%
%
% INPUTS
%
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
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
% eak -------- Commanded 4x1 voltage vector to be applied at time tk, in
%              volts. eak(i) is the voltage for the ith motor.
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
  ip.addRequired('Fk',@(x)issize(x,1,1));
  ip.addRequired('NBk',@(x)issize(x,3,1));
  ip.addParameter('quadParams',[],@(x)isstruct(x));
  ip.addParameter('constants',[],@(x)isstruct(x));
  ip.parse(Fk,NBk,P);
end

%% Student code

% Unpack P
qp = P.quadParams;
c = P.contants;

% Initial resource allocation tuning values
alpha = 1;
beta = 0.9;


% Construct G matrix
kT = qp.kN ./ qp.kF;
G = [ones(1, 4); qp.rotor_loc(2, :); -1 * qp.rotor_loc(1, :); kT*qp.omega_dir];

% Max thrust from a single motor
Fimax = qp.kF*(qp.cm.*qp.eamax).^2;
FVec = Fimax + 1; % Ensure the loop runs the first time

% Save thrust for attitude control
Fapp = min([Fk, sum(Fimax)*beta]);

while any(FVec > Fimax)

    % Desired actuation
    star = [Fapp; alpha*NBk];
    
    % Solve for motor thrust
    FVec = G\star;

    alpha = 0.9*alpha;
end

% Zero out any negative values
FVec = max(FVec, 0);

% Convert motor thrust to applied voltage
eak = ((FVec./qp.kF).^0.5)./qp.cm;

end % EOF voltageConverter.m