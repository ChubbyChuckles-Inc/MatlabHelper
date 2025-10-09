%% Lecture 7 — Practice Tasks on Singularities and Damped Least Squares
% Prerequisite: ensure your Lecture 7 functions (manipulability, is_singular,
% dls_step, ik_step) are on the MATLAB path together with any dependencies
% (such as jacobian_geometric, transl, rotZ). Tasks progress from evaluating
% manipulability (Task 1) to performing multi-step IK convergence (Task 5).

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Manipulability at a nominal pose
% Goal: Compute the manipulability of a 3R manipulator and interpret the
% resulting scalar.
fprintf('\n[Task 1] Step 1: Specify robot model and joint vector.\n');
dh_table = [
    0.30, 0, 0.08, 0;
    0.25, 0, 0.00, 0;
    0.15, 0, 0.00, 0
];
q = deg2rad([10, -40, 30]);
[J, ~] = jacobian_geometric(dh_table, q, false);
manip = manipulability(J);
fprintf('Manipulability measure w = %.4f\n', manip);

%% Task 2 — Detect near-singular configurations
% Goal: Sweep joint 2 toward a straight configuration and flag where
% is_singular triggers for tolerance 1e-3.
fprintf('\n[Task 2] Step 1: Sweep q2 and evaluate singularity flag.\n');
q2_vals = deg2rad(linspace(-90, 40, 27));
for angle = q2_vals
    q_test = [q(1), angle, q(3)];
    [J_test, ~] = jacobian_geometric(dh_table, q_test, false);
    flag = is_singular(J_test, 1e-3);
    fprintf('q2 = %+5.1f° → singular? %d\n', rad2deg(angle), flag);
end

%% Task 3 — Apply a single damped least-squares update
% Goal: Given a target Cartesian error, compute Δq using dls_step and observe
% how damping changes the step norm.
fprintf('\n[Task 3] Step 1: Construct target pose and error twist.\n');
T_current = fk_from_jacobian(dh_table, q);
T_target = transl(0.38, 0.05, 0.24) * rotZ(deg2rad(30));
err = pose_error(T_current, T_target);
fprintf('[Task 3] Step 2: Compare small vs large damping.\n');
Delta_small = dls_step(J, err, 0.02);
Delta_large = dls_step(J, err, 0.20);
fprintf('‖Δq‖ (λ=0.02) = %.3f rad, (λ=0.20) = %.3f rad\n', norm(Delta_small), norm(Delta_large));

%% Task 4 — Execute one IK step toward the target
% Goal: Use ik_step to update q and observe the residual reduction.
fprintf('\n[Task 4] Step 1: Perform a single IK update.\n');
[q_next, residual] = ik_step(dh_table, q, T_target, 0.05, deg2rad(8));
fprintf('Residual after step: %.3e, updated q = [%.2f %.2f %.2f]°\n', residual, rad2deg(q_next));

%% Task 5 — Iterate until convergence and track manipulability
% Goal: Apply ik_step iteratively until the residual drops below 1e-4, logging
% manipulability at each iteration.
fprintf('\n[Task 5] Step 1: Iterate IK with manipulability logging.\n');
max_iters = 20;
q_iter = q;
for iter = 1:max_iters
    [q_iter, residual] = ik_step(dh_table, q_iter, T_target, 0.05, deg2rad(8));
    [J_iter, ~] = jacobian_geometric(dh_table, q_iter, false);
    manip_iter = manipulability(J_iter);
    fprintf('Iter %02d → residual=%.2e, manipulability=%.4f\n', iter, residual, manip_iter);
    if residual < 1e-4
        fprintf('Converged in %d iterations!\n', iter);
        break;
    end
end
if residual >= 1e-4
    warning('Residual remained %.2e after %d iterations.', residual, max_iters);
end

%% Local helpers mirroring lecture utilities
function T = fk_from_jacobian(dh_table, q)
Ts_local = eye(4);
for i = 1:size(dh_table,1)
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts_local = Ts_local * dh(a, alpha, d, theta);
end
T = Ts_local;
end

function err = pose_error(T_current, T_target)
linear = T_target(1:3,4) - T_current(1:3,4);
R_err = T_current(1:3,1:3)' * T_target(1:3,1:3);
angle = acos(max(min((trace(R_err) - 1)/2, 1), -1));
if angle < 1e-8
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
