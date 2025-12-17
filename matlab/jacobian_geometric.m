function [J, Ts] = jacobian_geometric(dh_table, q, verbose)
%JACOBIAN_GEOMETRIC Geometric Jacobian for a serial chain of revolute joints.
%   dh_table: N×4 [a alpha d theta_offset]
%   q: 1×N or N×1 joint angles.
%   J: 6×N mapping qdot -> [v; ω]
%   Ts: (N+1)×1 cell of transforms T_0^i.

if nargin < 3
    verbose = false;
end

[Ts, ~] = chainDH(dh_table, q, false);

n = size(dh_table, 1);
J = zeros(6, n);

o_n = Ts{end}(1:3, 4);
for i = 1:n
    z_i = Ts{i}(1:3, 3);
    o_i = Ts{i}(1:3, 4);
    J(1:3, i) = cross(z_i, o_n - o_i);
    J(4:6, i) = z_i;
end

if verbose
    fprintf('rank(J) = %d, cond(J) = %.2f\n', rank(J), cond(J));
end
end
