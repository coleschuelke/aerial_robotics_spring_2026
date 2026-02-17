function [rpGtilde,rbGtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
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
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%
% OUTPUTS
%
% rpGtilde --- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rbGtilde --- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rbGtilde is constrained to satisfy norm(rbGtilde) = b, where b
%              is the known baseline distance between the two antennas.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Quentin Cole Schuelke
%+==============================================================================+  

%% Validate inputs
global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addParameter('rI',[],@(x)issize(x,3,1));
  ip.addParameter('RBI',[],@(x)issize(x,3,3));
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(S.statek,P);
end

%% Student code
% Unpack state
rI = S.statek.rI;
RBI = S.statek.RBI;

% Params
sp = P.sensorParams;

% Convenience definitions
ra1B = sp.raB(:, 1);
ra2B = sp.raB(:, 2);

% Common math
ra1I = RBI.'*ra1B;
ra2I = RBI.'*ra2B;
RLG = Recef2enu(sp.r0G);
RpG = RLG*sp.RpL*RLG.';

% rpG measurement
rpI = rI + ra1I;
rpG = RLG.'*rpI;
rpGtilde = rpG + mvnrnd(zeros(3, 1), RpG).';

% rbG measurement
rsI = rI + ra2I;
rsG = RLG.'*rsI;
rbG = rsG - rpG;

% Special rank-2 cov for baseline noise
eps = 1e-8;
rbGu = rbG/norm(rbG);
RbG = norm(rbG)^2*sp.sigmab^2*(eye(3) - rbGu*rbGu.') + eps*eye(3);

rbGtilde = rbG + mvnrnd(zeros(3, 1), RbG).';

end % EOF gnssMeasSimulator.m