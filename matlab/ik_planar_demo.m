%% Lecture 5 Demo — Exploring planar IK solutions
% Run each section to compare multiple IK targets and branches with
% immediate textual feedback.

clearvars;
clc;

%% 1. Enumerate targets for a 2R arm (l₁=0.35 m, l₂=0.25 m)
fprintf('\n=== Step 1: Evaluate a set of workspace targets ===\n');
L1 = 0.35; L2 = 0.25;
targets = [
    0.40,  0.20;
    0.40, -0.15;
    0.10,  0.45;
    0.70,  0.00  % near reach limit
];
for idx = 1:size(targets,1)
    xy = targets(idx,:);
    fprintf('\nTarget (x,y) = (%.2f, %.2f) m\n', xy(1), xy(2));
    q_up = ik2R(xy(1), xy(2), L1, L2, 'up');
    q_down = ik2R(xy(1), xy(2), L1, L2, 'down');
    validate2R(xy(1), xy(2), L1, L2, 'up');
    validate2R(xy(1), xy(2), L1, L2, 'down');
end

%% 2. Show unreachable case explicitly
fprintf('\n=== Step 2: Unreachable pose ===\n');
ik2R(1.0, 0.4, L1, L2, 'down');

%% Local helper functions (duplicated for demo self-containment)

function [q, feasible] = ik2R(x, y, l1, l2, branch)
if nargin < 5, branch = 'down'; end
r2 = x^2 + y^2;
reach = l1 + l2;
if sqrt(r2) > reach + 1e-9
    warning('Target (%.3f, %.3f) lies outside workspace (%.3f m).', x, y, reach);
    q = [NaN, NaN];
    feasible = false;
    return;
end
cos_q2 = (r2 - l1^2 - l2^2) / (2 * l1 * l2);
cos_q2 = max(min(cos_q2, 1), -1);
if strcmpi(branch, 'up')
    q2 = acos(cos_q2);
else
    q2 = -acos(cos_q2);
end
k1 = l1 + l2 * cos(q2);
k2 = l2 * sin(q2);
q1 = atan2(y, x) - atan2(k2, k1);
q = [q1, q2];
feasible = true;
fprintf('ik2R (%s) → q=[%.2f %.2f]°\n', branch, q1*180/pi, q2*180/pi);
end

function validate2R(x, y, l1, l2, branch)
[q, feasible] = ik2R(x, y, l1, l2, branch);
if ~feasible, return; end
fk = fk_planar(q, l1, l2);
residual = norm(fk - [x; y]);
fprintf('  residual_%s = %.3e m\n', branch, residual);
end

function p = fk_planar(q, l1, l2)
x = l1 * cos(q(1)) + l2 * cos(q(1) + q(2));
y = l1 * sin(q(1)) + l2 * sin(q(1) + q(2));
p = [x; y];
end
