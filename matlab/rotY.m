function T = rotY(theta, verbose)
%ROTY Homogeneous rotation about Y axis.
%   T = rotY(theta) returns a 4x4 homogeneous rotation.
%   T = rotY(theta, verbose) prints a short message if verbose=true.

if nargin < 2
    verbose = false;
end

T = eye(4);
T([1 3],[1 3]) = [cos(theta), sin(theta); -sin(theta), cos(theta)];

if verbose
    fprintf('rotY(%.3f rad) created.\n', theta);
end
end
