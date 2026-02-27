function [RBI] = wahbaSolver(aVec,vIMat,vBMat)
% wahbaSolver : Solves Wahba's problem via SVD.  In other words, this
%               function finds the rotation matrix RBI that minimizes the
%               cost Jw:
%
%                     N
%    Jw(RBI) = (1/2) sum ai*||viB - RBI*viI||^2
%                    i=1
%
%
% INPUTS
%
% aVec ------- Nx1 vector of least-squares weights.  aVec(i) is the weight
%              corresponding to the ith pair of vectors 
%
% vIMat ------ Nx3 matrix of 3x1 unit vectors expressed in the I frame.
%              vIMat(i,:)' is the ith 3x1 vector.
%
% vBMat ------ Nx3 matrix of 3x1 unit vectors expressed in the B
%              frame. vBMat(i,:)' is the ith 3x1 vector, which corresponds to
%              vIMat(i,:)';
%
% OUTPUTS
% 
% RBI -------- 3x3 direction cosine matrix indicating the attitude of the
%              B frame relative to the I frame.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Quentin Cole Schuelke
%+==============================================================================+  

global INPUT_PARSING;
if INPUT_PARSING
  issize =@(x,z1,z2) validateattributes(x,{'numeric'},{'size',[z1,z2]});
  ip = inputParser; ip.StructExpand = true; ip.KeepUnmatched = true;
  ip.addRequired('aVec',@(x)issize(x,NaN,1));
  ip.addRequired('vIMat',@(x)issize(x,NaN,3));
  ip.addRequired('vBMat',@(x)issize(x,NaN,3));
  ip.parse(aVec,vIMat,vBMat);
end

%% Student code
B = zeros(3);
for i=1:length(aVec)
    B = B + aVec(i)*vBMat(i)*vIMat(i).';
    [U, ~, V] = svd(B);
    RBI = U*diag([1, 1, det(U)*det(V)])*V.';
end

end % EOF wahbaSolver.m