function A = dh(a, alpha, d, theta, verbose)
%DH Standard Denavit-Hartenberg homogeneous transform.
%   A = dh(a, alpha, d, theta) returns a 4x4 transform.
%   A = dh(..., verbose) prints the matrix if verbose=true.

if nargin < 5
    verbose = false;
end

A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,             sin(alpha),             cos(alpha),            d;
              0,                     0,                     0,            1];

if verbose
    fprintf('dh(a=%.3f, α=%.3f, d=%.3f, θ=%.3f rad) ->\n', a, alpha, d, theta);
    disp(A);
end
end
