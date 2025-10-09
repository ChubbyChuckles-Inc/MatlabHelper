%% Lecture 5 — Analytic IK for a planar 2R arm
% Build inverse kinematics utilities section-by-section. Each helper prints
% intermediate insight to the Command Window.

clearvars;
clc;

%% 1. Solve IK for a reachable point (elbow-down)
fprintf('\n=== Step 1: Solve IK for (x,y) = (0.4, 0.2) ===\n');
[q_elbowDown, feasible] = ikine_2R(0.4, 0.2, 0.35, 0.25, 'down');
fprintf('Solution (deg): [%.2f %.2f], feasible=%d\n', q_elbowDown*180/pi, feasible);
validate_ik_2R(0.4, 0.2, 0.35, 0.25, 'down');

%% 2. Compare elbow-up vs elbow-down branches
fprintf('\n=== Step 2: Compare solution branches ===\n');
[q_up, feasible_up] = ikine_2R(0.4, 0.2, 0.35, 0.25, 'up');
fprintf('Elbow-up (deg):   [%.2f %.2f]\n', q_up*180/pi);
fprintf('Elbow-down (deg): [%.2f %.2f]\n', q_elbowDown*180/pi);

%% 3. Attempt an unreachable target
fprintf('\n=== Step 3: Attempt unreachable target ===\n');
ikine_2R(1.2, 0.0, 0.35, 0.25, 'down');

%% Local helper functions

function [q, feasible] = ikine_2R(x, y, l1, l2, branch)
if nargin < 5, branch = 'down'; end
r2 = x^2 + y^2;
reach = l1 + l2;
if sqrt(r2) > reach + 1e-9
    warning('Target (%.3f, %.3f) lies outside the reachable workspace (%.3f).', x, y, reach);
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
fprintf('ikine_2R → branch=%s, q1=%.2f°, q2=%.2f°\n', branch, q1*180/pi, q2*180/pi);
end

function validate_ik_2R(x, y, l1, l2, branch)
if nargin < 5, branch = 'down'; end
[q, feasible] = ikine_2R(x, y, l1, l2, branch);
if ~feasible
    fprintf('Validation skipped (unreachable pose).\n');
    return;
end
fk = planar_fk(q, l1, l2);
residual = norm(fk - [x; y]);
fprintf('validate_ik_2R residual: %.3e m\n', residual);
end

function p = planar_fk(q, l1, l2)
x = l1 * cos(q(1)) + l2 * cos(q(1) + q(2));
y = l1 * sin(q(1)) + l2 * sin(q(1) + q(2));
p = [x; y];
end
