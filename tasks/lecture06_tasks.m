%% Lecture 6 — Practice Tasks on Differential Kinematics
% Prerequisite: add jacobian_geometric (and related helpers) from Lecture 6
% to the MATLAB path. Tasks escalate from computing a Jacobian (Task 1) to
% validating velocity predictions and singularity metrics (Task 5).

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Compute the Jacobian for a 3R arm
% Goal: Evaluate jacobian_geometric at q = [15°, -35°, 20°] and inspect the
% resulting matrix.
fprintf('\n[Task 1] Step 1: Specify the robot and joint vector.\n');
dh_table = [
    0.30, 0, 0.08, 0;
    0.25, 0, 0.00, 0;
    0.15, 0, 0.00, 0
];
q = deg2rad([15, -35, 20]);
fprintf('[Task 1] Step 2: Compute the Jacobian.\n');
[J, Ts] = jacobian_geometric(dh_table, q, true);
fprintf('Jacobian (6×3):\n');
disp(J);

%% Task 2 — Map joint rates to spatial twist
% Goal: Multiply the Jacobian by q̇ = [0.1; -0.2; 0.15] and interpret the
% resulting spatial velocity components.
fprintf('\n[Task 2] Step 1: Define the joint-rate vector.\n');
qdot = [0.1; -0.2; 0.15];
fprintf('[Task 2] Step 2: Compute spatial twist = J*qdot.\n');
twist = J * qdot;
fprintf('Linear velocity (m/s): [%.3f %.3f %.3f]\n', twist(1:3));
fprintf('Angular velocity (rad/s): [%.3f %.3f %.3f]\n', twist(4:6));

%% Task 3 — Finite-difference validation of translational velocity
% Goal: Apply a small joint increment Δq and compare the predicted Cartesian
% displacement (using J) with actual FK displacement.
fprintf('\n[Task 3] Step 1: Select Δt = 0.01 s and compute Δq.\n');
dt = 0.01;
Delta_q = qdot * dt;
fprintf('[Task 3] Step 2: Predict Δx ≈ J(1:3,:) * Δq.\n');
Delta_x_pred = J(1:3,:) * Delta_q;
fprintf('Predicted Δx: [%.5f %.5f %.5f] m\n', Delta_x_pred);
fprintf('[Task 3] Step 3: Compare with FK evaluation.\n');
T_before = Ts{end};
q_after = q + Delta_q';
T_after = jacobian_fk(dh_table, q_after);
Delta_x_actual = T_after(1:3,4) - T_before(1:3,4);
fprintf('Actual Δx: [%.5f %.5f %.5f] m\n', Delta_x_actual);
fprintf('Discrepancy norm: %.3e m\n', norm(Delta_x_actual - Delta_x_pred));

%% Task 4 — Examine rotational velocity consistency
% Goal: Verify that the predicted angular velocity integrates to the observed
% change in orientation over Δt.
fprintf('\n[Task 4] Step 1: Integrate angular velocity over Δt.\n');
omega = twist(4:6);
R_before = T_before(1:3,1:3);
R_pred = R_before * skewexp(omega * dt);
R_actual = T_after(1:3,1:3);
rot_error = acos(max(min((trace(R_pred' * R_actual) - 1)/2, 1), -1));
fprintf('Orientation prediction error: %.3e rad\n', rot_error);

%% Task 5 — Characterise manipulability and near-singularity
% Goal: Compute manipulability measure √det(J Jᵀ) and condition number as the
% arm approaches a stretched configuration. Identify when the Jacobian is near
% singular.
fprintf('\n[Task 5] Step 1: Sweep q2 toward extension.\n');
q2_sweep = deg2rad(linspace(-90, 30, 25));
for angle = q2_sweep
    [J_tmp, ~] = jacobian_geometric(dh_table, [q(1), angle, q(3)], false);
    sigma = svd(J_tmp);
    w = prod(sigma);
    condJ = max(sigma) / min(sigma);
    fprintf('q2 = %+5.1f° → manipulability = %.4f, cond = %.2f\n', rad2deg(angle), w, condJ);
end

%% Local helpers mirroring lecture derivations
function T = jacobian_fk(dh_table, q)
% Compute FK using repeated dh() calls (no verbose prints).
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

function R = skewexp(omega)
% Rodrigues formula for integrating a small rotation vector omega.
theta = norm(omega);
if theta < 1e-8
    R = eye(3) + skew(omega);
else
    k = omega / theta;
    K = skew(k);
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end
end

function S = skew(v)
S = [  0,   -v(3),  v(2);
     v(3),    0,   -v(1);
    -v(2),  v(1),    0  ];
end

function T = dh(a, alpha, d, theta)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,             sin(alpha),             cos(alpha),            d;
            0,                     0,                     0,            1];
end
