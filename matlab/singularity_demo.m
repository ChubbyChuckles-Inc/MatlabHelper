%% Lecture 7 Demo — Navigating singularities with damped IK
% Compare naive and damped updates near a singular configuration.

clearvars;
clc;

%% 1. Setup: arm straightened to highlight singular behaviour
fprintf('\n=== Step 1: Initialise near-singular posture ===\n');
dh_table = [...
    0.3, 0, 0.1, 0;
    0.25, 0, 0.0, 0;
    0.15, 0, 0.0, 0
];
q0 = [0, 0, 0];
[J0, Ts0] = jacobian_geometric(dh_table, q0);
manip = manipulability(J0);
fprintf('Initial manipulability w = %.4e\n', manip);

%% 2. Target pose slightly offset from singular direction
T_target = transl(0.65, 0.02, 0.1) * rotZ(5*pi/180);

%% 3. Iterations without damping
fprintf('\n=== Step 3: Iterative IK without damping ===\n');
q = q0;
for iter = 1:5
    [T_current, J] = fk_and_jacobian(dh_table, q);
    err = pose_error(T_current, T_target);
    delta_q = pinv(J) * err;
    q = q + delta_q.';
    fprintf('Iter %d (λ=0): ‖Δq‖=%.3f, residual=%.3e\n', iter, norm(delta_q), norm(err));
end

%% 4. Iterations with damped least squares
fprintf('\n=== Step 4: Iterative IK with damping (λ=0.1) ===\n');
q = q0;
for iter = 1:5
    [q, residual] = ik_step(dh_table, q, T_target, 0.1, deg2rad(8));
    fprintf('Iter %d (λ=0.1): residual=%.3e\n', iter, residual);
end

%% Local helper functions (self-contained)

function [q_next, residual_norm] = ik_step(dh_table, q, T_target, lambda, max_step)
if nargin < 4, lambda = 0.05; end
if nargin < 5, max_step = deg2rad(10); end
[T_current, J] = fk_and_jacobian(dh_table, q);
err = pose_error(T_current, T_target);
JT = J';
delta_q = JT * ((J * JT + (lambda^2) * eye(6)) \ err);
if norm(delta_q) > max_step
    delta_q = delta_q * (max_step / norm(delta_q));
    fprintf('Step clipped to %.2f° magnitude.\n', rad2deg(max_step));
end
q_next = q + delta_q.';
residual_norm = norm(err);
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
