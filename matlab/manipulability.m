function w = manipulability(J)
%MANIPULABILITY Yoshikawa manipulability measure.
%   w = sqrt(det(J*J')) for m×n Jacobian J.
%   Returns 0 if numerical roundoff produces a tiny negative determinant.

M = J * J.';
d = det(M);
if d < 0 && abs(d) < 1e-12
    d = 0;
end
if d < 0
    % Fallback to a robust computation via singular values
    s = svd(J);
    w = prod(s);
    return;
end
w = sqrt(d);
end
