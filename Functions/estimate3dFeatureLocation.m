function [rXIHat,Px] = estimate3dFeatureLocation(M,P)
% estimate3dFeatureLocation : Estimate the 3D coordinates of a feature point
%                             seen by two or more cameras with known pose.
%
%
% INPUTS
%
% M ---------- Structure with the following elements:
%
%       rxArray = 1xN cell array of measured positions of the feature point
%                 projection on the camera's image plane, in pixels.
%                 rxArray{i} is the 2x1 vector of coordinates of the feature
%                 point as measured by the ith camera.  To ensure the
%                 estimation problem is observable, N must satisfy N >= 2 and
%                 at least two cameras must be non-colinear.
%
%      RCIArray = 1xN cell array of I-to-camera-frame attitude matrices.
%                 RCIArray{i} is the 3x3 attitude matrix corresponding to the
%                 measurement rxArray{i}.
%
%       rcArray = 1xN cell array of camera center positions.  rcArray{i} is
%                 the 3x1 position of the camera center corresponding to the
%                 measurement rxArray{i}, expressed in the I frame in meters.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the quad's
%                 sensors, as defined in sensorParamsScript.m
%
% OUTPUTS
%
%
% rXIHat -------- 3x1 estimated location of the feature point expressed in I
%                 in meters.
%
% Px ------------ 3x3 error covariance matrix for the estimate rxIHat.
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
  ip.addParameter('rxArray',[],@(x)issize(x{1},2,1));
  ip.addParameter('RCIArray',[],@(x)issize(x{1},3,3));
  ip.addParameter('rcArray',[],@(x)issize(x{1},3,1));
  ip.addParameter('sensorParams',[],@(x)isstruct(x));
  ip.parse(M,P);
end

%% Student code
sp = P.sensorParams;

N = length(M.rxArray);
Hprime = zeros(2*N, 4);
R = sp.pixelSize^2 * kron(eye(N), sp.Rc);
Rinv = inv(R);

for i=1:N
    RCI = M.RCIArray{i};
    tC = RCI*M.rcArray{i};
    Pi = sp.K*[RCI , -tC];
    p1T = Pi(1, :);
    p2T = Pi(2, :);
    p3T = Pi(3, :);
    
    % Convert from pixels to m
    xtilde = sp.pixelSize*M.rxArray{i}(1);
    ytilde = sp.pixelSize*M.rxArray{i}(2);

    Hprime(2*i - 1, :) = xtilde*p3T - p1T;
    Hprime(2*i, :) = ytilde*p3T - p2T;
end

H = Hprime(:, 1:3);
z = -1 * Hprime(:, 4);

Px = inv(H.'*Rinv*H);
rXIHat = Px*H.'*Rinv*z;
  
end % EOF estimate3dFeatureLocation.m