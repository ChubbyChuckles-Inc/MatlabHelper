function T = transl(x, y, z, verbose)
%TRANSL Homogeneous translation.
%   T = transl(x,y,z) returns a 4x4 translation matrix.
%   T = transl(x,y,z,verbose) prints a short message if verbose=true.

if nargin < 4
    verbose = false;
end

T = eye(4);
T(1:3,4) = [x; y; z];

if verbose
    fprintf('transl([%.3f %.3f %.3f]) created.\n', x, y, z);
end
end
