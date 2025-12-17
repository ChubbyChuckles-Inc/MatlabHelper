function T = rotX(theta, verbose)
%ROTX Homogeneous rotation about X axis.
%   T = rotX(theta) returns a 4x4 homogeneous rotation.
%   T = rotX(theta, verbose) prints a short message if verbose=true.

if nargin < 2
    verbose = false;
end

T = eye(4);
T(2:3,2:3) = [cos(theta), -sin(theta); sin(theta), cos(theta)];

if verbose
    fprintf('rotX(%.3f rad) created.\n', theta);
end
end
