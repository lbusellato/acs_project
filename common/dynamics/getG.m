%--------------------------------------------------------------------------
%
% getG.m
%
% This script computes numerically the gravity vector.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function g = getG(q)
    q3 = q(3);
    g = [ 0
          0
         -0.9621*cos(q3)];
end