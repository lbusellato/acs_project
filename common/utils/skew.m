%--------------------------------------------------------------------------
%
%   skew.m
%
% This script computes the skew-symmetric matrix for a given vector.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function S = skew(v)
    S = [0 -v(3) v(2);
        v(3)  0 -v(1);
        -v(2) v(1) 0];
end