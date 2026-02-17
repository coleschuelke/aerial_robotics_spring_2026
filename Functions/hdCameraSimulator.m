function [rx] = hdCameraSimulator(rXI,S,P)
% hdCameraSimulator : Simulates feature location measurements from the
%                     quad's high-definition camera. 
%
%
% INPUTS
%
% rXI -------- 3x1 location of a feature point expressed in I in meters.
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
% OUTPUTS
%
% rx --------- 2x1 measured position of the feature point projection on the
%              camera's image plane, in pixels.  If the feature point is not
%              visible to the camera (the ray from the feature to the camera
%              center never intersects the image plane, or the feature is
%              behind the camera), then rx is an empty matrix.
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
  ip.addRequired('rXI',@(x)issize(x,3,1));
  ip.addParameter('rI',[],@(x)issize(x,3,1))
  ip.addParameter('RBI',[],@(x)issize(x,3,3))
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(rXI,S.statek,P);
end

%% Student code

% Unpack S
rI = S.statek.rI;
RBI = S.statek.RBI;

% Unpack P
sp = P.sensorParams;

% Compute RCI
RCI = sp.RCB*RBI;

% Compute tC
tI = rI + RBI.'*sp.rocB;
tC = RCI*tI;

% Compute homogenous coords on the camera plane
P = [RCI, -tC];
rXIh = [rXI; 1];
xh = sp.K*P*rXIh;

if xh(3) <= 0 % Behind us, we def can't see
    rx = [];
    return
end

% Compute pixel coordinates on the image plane
x = [xh(1)/xh(3);xh(2)/xh(3)]; % m

% Check whether object is visible
if any(abs(x) > sp.imagePlaneSize.')
    rx = [];
else
    rx = 1/sp.pixelSize * x + mvnrnd(zeros(2, 1), sp.Rc).'; % Pixels


end 