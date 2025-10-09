%% Lecture 6 Demo — Validating the geometric Jacobian
% This script performs numerical experiments to confirm the relationship
% v = J(q)·q̇ and to explore how individual joint motions influence the
% end-effector twist.

clearvars;
clc;

%% 1. Setup manipulator and compute baseline FK/Jacobian
fprintf('\n=== Step 1: Baseline configuration ===\n');
dh_table = [...
    0.3, 0, 0.1, 0;
    0.25, 0, 0.0, 0;
    0.15, 0, 0.0, 0
];
q = [20, -35, 40] * pi/180;
[J, Ts] = jacobian_geometric(dh_table, q);
T = Ts{end};
tprint(T, 'Baseline pose');

%% 2. Numerical differentiation check
fprintf('\n=== Step 2: Compare J·q̇ with finite difference ===\n');
qdot = [0.3; -0.2; 0.1];
dt = 1e-3;
T_plus = fkine(dh_table, q + qdot.' * dt);
finite_twist = twist_from_transform(T, T_plus, dt);
analytic_twist = J * qdot;
fprintf('Finite-difference twist: [%.3f %.3f %.3f | %.3f %.3f %.3f]^T\n', finite_twist);
fprintf('Analytic (J·q̇):         [%.3f %.3f %.3f | %.3f %.3f %.3f]^T\n', analytic_twist);
fprintf('‖difference‖₂ = %.3e\n', norm(finite_twist - analytic_twist));

%% 3. Effect of single joint rate perturbations
fprintf('\n=== Step 3: Single-joint perturbations ===\n');
for joint = 1:3
    unit_qdot = zeros(3,1); unit_qdot(joint) = 1.0;
    twist_axis = J * unit_qdot;
    fprintf('Joint %d unit rate → v=[%.3f %.3f %.3f], ω=[%.3f %.3f %.3f]\n', ...
        joint, twist_axis(1:3), twist_axis(4:6));
end

%% Local helper functions (self-contained)

function [J, Ts] = jacobian_geometric(dh_table, q)
n = size(dh_table, 1);
Ts = cell(n+1, 1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts{i+1} = Ts{i} * dh(a, alpha, d, theta);
end
J = zeros(6, n);
o_n = Ts{end}(1:3,4);
for i = 1:n
    z = Ts{i}(1:3,3);
    o = Ts{i}(1:3,4);
    J(1:3,i) = cross(z, o_n - o);
    J(4:6,i) = z;
end
fprintf('rank(J) = %d, cond(J) = %.2f\n', rank(J), cond(J));
end

function T = fkine(dh_table, q)
n = size(dh_table, 1);
T = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    T = T * dh(a, alpha, d, theta);
end
end

function twist = twist_from_transform(T, T_plus, dt)
R = T(1:3,1:3);
Rp = T_plus(1:3,1:3);
p = T(1:3,4);
pp = T_plus(1:3,4);
linear = (pp - p) / dt;
R_rel = R' * Rp;
angle = acos(max(min((trace(R_rel) - 1)/2, 1), -1));
if abs(angle) < 1e-12
    axis = [0; 0; 0];
else
    axis = (1/(2*sin(angle))) * [R_rel(3,2) - R_rel(2,3);
                                  R_rel(1,3) - R_rel(3,1);
                                  R_rel(2,1) - R_rel(1,2)];
end
angular = R * axis * angle / dt;
twist = [linear; angular];
end

function T = dh(a, alpha, d, theta)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,             sin(alpha),             cos(alpha),            d;
              0,                     0,                     0,            1];
end

function tprint(T, label)
if nargin < 2, label = 'Transform'; end
fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f] m\n', T(1:3,4));
fprintf('Rotation:\n');
disp(T(1:3,1:3));
end
