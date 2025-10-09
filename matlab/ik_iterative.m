%% Lecture 7 — Singularities and damped least-squares IK steps
% Build tools for recognising singularities and performing robust iterative
% IK updates.

clearvars;
clc;

%% 1. Compute manipulability and singularity checks
fprintf('\n=== Step 1: Manipulability measure ===\n');
dh_table = [...
    0.3, 0, 0.1, 0;
    0.25, 0, 0.0, 0;
    0.15, 0, 0.0, 0
];
q = [10, -50, 40] * pi/180;
[J, ~] = jacobian_geometric(dh_table, q);
manip = manipulability(J);
flag = is_singular(J, 1e-4);
fprintf('Manipulability w = %.4f, singular=%d\n', manip, flag);

%% 2. Execute one damped least-squares step toward a target pose
fprintf('\n=== Step 2: Damped least-squares IK step ===\n');
T_target = transl(0.4, 0.05, 0.25) * rotZ(pi/4);
[q_next, residual] = ik_step(dh_table, q, T_target, 0.05, deg2rad(5));
fprintf('Residual norm after step: %.3e\n', residual);

%% Local helper functions

function w = manipulability(J)
sigma = svd(J);
w = prod(sigma);
fprintf('manipulability → %.4f\n', w);
end

function flag = is_singular(J, tol)
w = manipulability(J);
flag = w < tol;
if flag
    fprintf('*** Jacobian is near-singular (w=%.4e < %.1e). ***\n', w, tol);
else
    fprintf('Jacobian is well-conditioned (w=%.4e ≥ %.1e).\n', w, tol);
end
end

function delta_q = dls_step(J, err, lambda)
JT = J';
delta_q = JT * ((J * JT + (lambda^2) * eye(6)) \ err);
fprintf('dls_step → ‖Δq‖₂ = %.3f (λ=%.3f)\n', norm(delta_q), lambda);
end

function [q_next, residual_norm] = ik_step(dh_table, q, T_target, lambda, max_step)
if nargin < 4, lambda = 0.05; end
if nargin < 5, max_step = deg2rad(10); end
[T_current, J] = fk_and_jacobian(dh_table, q);
err = pose_error(T_current, T_target);
delta_q = dls_step(J, err, lambda);
if norm(delta_q) > max_step
    delta_q = delta_q * (max_step / norm(delta_q));
    fprintf('Step clipped to %.2f° max magnitude.\n', rad2deg(max_step));
end
q_next = q + delta_q.';
residual_norm = norm(err);
fprintf('ik_step → residual=%.3e\n', residual_norm);
end

function [T, J] = fk_and_jacobian(dh_table, q)
[J, Ts] = jacobian_geometric(dh_table, q);
T = Ts{end};
end

function [J, Ts] = jacobian_geometric(dh_table, q)
n = size(dh_table,1);
Ts = cell(n+1,1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts{i+1} = Ts{i} * dh(a, alpha, d, theta);
end
J = zeros(6,n);
o_n = Ts{end}(1:3,4);
for i = 1:n
    z = Ts{i}(1:3,3);
    o = Ts{i}(1:3,4);
    J(1:3,i) = cross(z, o_n - o);
    J(4:6,i) = z;
end
end

function err = pose_error(T_current, T_target)
linear = T_target(1:3,4) - T_current(1:3,4);
R_err = T_current(1:3,1:3)' * T_target(1:3,1:3);
angle = acos(max(min((trace(R_err) - 1)/2, 1), -1));
if abs(angle) < 1e-8
    axis = zeros(3,1);
else
    axis = (1/(2*sin(angle))) * [R_err(3,2) - R_err(2,3);
                                  R_err(1,3) - R_err(3,1);
                                  R_err(2,1) - R_err(1,2)];
end
angular = T_current(1:3,1:3) * axis * angle;
err = [linear; angular];
end

function T = transl(x, y, z)
T = eye(4);
T(1:3,4) = [x; y; z];
end

function T = rotZ(theta)
T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end

function T = dh(a, alpha, d, theta)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,             sin(alpha),             cos(alpha),            d;
              0,                     0,                     0,            1];
end
