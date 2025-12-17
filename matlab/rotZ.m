function T = rotZ(theta, verbose)
%ROTZ Homogeneous rotation about Z axis.
%   T = rotZ(theta) returns a 4x4 homogeneous rotation.
%   T = rotZ(theta, verbose) prints a short message if verbose=true.

if nargin < 2
    verbose = false;
end

T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];

if verbose
    fprintf('rotZ(%.3f rad) created.\n', theta);
end
end
