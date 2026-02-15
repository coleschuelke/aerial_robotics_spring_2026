function [Xnew] = normalizeR(X)
%NORMALIZER Normalize rotation matrix R using SVD
%   Detailed explanation goes here

    R = [X(7:9), X(10:12), X(13:15)];
    [U, ~, V] = svd(R);
    R = U*V';

    Xnew = X;
    Xnew(7:15) = R(:);

end