%% Lecture 6 — Differential kinematics and the geometric Jacobian
% Build Jacobian utilities while observing intermediate calculations in the
% Command Window.

clearvars;
clc;

%% 1. Compute a Jacobian for a 3R manipulator
fprintf('\n=== Step 1: Geometric Jacobian at a sample configuration ===\n');
dh_table = [...
    0.3, 0, 0.1, 0;
    0.25, 0, 0.0, 0;
    0.15, 0, 0.0, 0
];
q = [10, -40, 30] * pi/180;
[J, Ts] = jacobian_geometric(dh_table, q, true);
fprintf('Jacobian (6×n):\n');
disp(J);

%% 2. Apply a sample joint velocity vector
fprintf('\n=== Step 2: Map joint rates to spatial twist ===\n');
qdot = [0.2; -0.1; 0.3];
twist = J * qdot;
fprintf('Resulting spatial velocity: [v; ω] = [%.3f %.3f %.3f | %.3f %.3f %.3f]^T\n', twist);

%% Local helper functions

function [J, Ts] = jacobian_geometric(dh_table, q, verbose)
if nargin < 3, verbose = false; end
n = size(dh_table, 1);
Ts = cell(n+1, 1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts{i+1} = Ts{i} * dh(a, alpha, d, theta);
    if verbose
        tprint(Ts{i+1}, sprintf('T_%d', i));
    end
end
J = zeros(6, n);
o_n = Ts{end}(1:3,4);
for i = 1:n
    z_i = Ts{i}(1:3,3);
    o_i = Ts{i}(1:3,4);
    J(1:3,i) = cross(z_i, o_n - o_i);
    J(4:6,i) = z_i;
end
rankJ = rank(J);
condJ = cond(J);
fprintf('rank(J) = %d, cond(J) = %.2f\n', rankJ, condJ);
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
