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
% Author:  
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

%                       Insert your code here 
  
end % EOF gnssMeasSimulator.m