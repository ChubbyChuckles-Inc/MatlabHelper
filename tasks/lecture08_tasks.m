%% Lecture 8 — Practice Tasks on Path Planning and Trajectory IK
% Prerequisite: ensure your Lecture 8 utilities (load_robot_dh, interpolate_pose,
% follow_path, ik_to_pose) and supporting functions (jacobian_geometric,
% manipulability) are on the MATLAB path. This script provides graceful
% fallbacks mirroring the lecture code if a function is missing, so you can
% compare your implementation against the reference behaviour.

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

% Resolve helpers, preferring student implementations when available.
load_robot_dh_fn = resolve_fn('load_robot_dh', @load_robot_dh_local);
interpolate_pose_fn = resolve_fn('interpolate_pose', @interpolate_pose_local);
follow_path_fn = resolve_fn('follow_path', @follow_path_local);
ik_to_pose_fn = resolve_fn('ik_to_pose', @ik_to_pose_local);
jacobian_fn = resolve_fn('jacobian_geometric', @jacobian_geometric_local);
manipulability_fn = resolve_fn('manipulability', @manipulability_local);

%% Task 1 — Inspect the robot model and joint limits
% Goal: Load the planar 3R robot description and print DH parameters and
% joint bounds.
fprintf('\n[Task 1] Step 1: Load robot data.\n');
[dh_table, q_limits] = load_robot_dh_fn('3R_planar');
disp(dh_table);
fprintf('[Task 1] Step 2: Report joint limits (deg).\n');
fprintf('Joint limits (deg):\n');
for i = 1:size(q_limits,1)
    fprintf('  q%d ∈ [%.1f, %.1f]\n', i, rad2deg(q_limits(i,1)), rad2deg(q_limits(i,2)));
end

%% Task 2 — Generate interpolated waypoints between poses
% Goal: Build ten waypoints between a start and goal pose using interpolate_pose.
fprintf('\n[Task 2] Step 1: Define start/goal and interpolate.\n');
T_start = transl(0.35, 0.05, 0.10);
T_goal = transl(0.55, -0.08, 0.15) * rotZ(deg2rad(70));
waypoints = interpolate_pose_fn(T_start, T_goal, 10);
fprintf('Generated %d waypoints. First and last positions:\n', numel(waypoints));
fprintf('  Start: [%.3f %.3f %.3f]\n', waypoints{1}(1:3,4));
fprintf('  End:   [%.3f %.3f %.3f]\n', waypoints{end}(1:3,4));

%% Task 3 — Follow the interpolated path via IK
% Goal: Starting from q0 = [0°, -20°, 45°], compute a joint trajectory that
% tracks the waypoint list.
fprintf('\n[Task 3] Step 1: Follow path with damped IK.\n');
q0 = deg2rad([0, -20, 45]);
q_traj = follow_path_fn(dh_table, q0, waypoints, 0.06);
fprintf('Trajectory samples: %d, using damping λ=0.06.\n', size(q_traj,1));

%% Task 4 — Reach a new pose with ik_to_pose
% Goal: For a target pose 20° rotated about Z at position [0.50, -0.05, 0.18],
% iteratively solve joint angles using ik_to_pose and check residual.
fprintf('\n[Task 4] Step 1: Define target and run ik_to_pose.\n');
T_target = transl(0.50, -0.05, 0.18) * rotZ(deg2rad(20));
q_guess = q_traj(end,:);
q_solution = ik_to_pose_fn(dh_table, q_guess, T_target, 0.05);
fprintf('Solution q (deg): [%.2f %.2f %.2f]\n', rad2deg(q_solution));
[T_solution, J_solution] = fk_and_jacobian_local(dh_table, q_solution);
residual = pose_error_local(T_solution, T_target);
fprintf('Final residual norm: %.3e\n', norm(residual));

%% Task 5 — Evaluate manipulability along the trajectory
% Goal: Use jacobian_geometric and manipulability to report dexterity along
% the previously computed path.
fprintf('\n[Task 5] Step 1: Compute manipulability along trajectory.\n');
for k = 1:size(q_traj,1)
    Jk = jacobian_fn(dh_table, q_traj(k,:), false);
    wk = manipulability_fn(Jk);
    fprintf('Waypoint %02d → manipulability %.4f\n', k, wk);
end

%% --- Helper functions --------------------------------------------------
function fn = resolve_fn(name, fallback)
if exist(name, 'file') || exist(name, 'builtin')
    fn = str2func(name);
else
    warning('Using fallback implementation for %s.', name);
    fn = fallback;
end
end

function [dh_table, q_limits] = load_robot_dh_local(name)
switch lower(name)
    case '3r_planar'
        dh_table = [
            0.30, 0, 0.00, 0;
            0.25, 0, 0.00, 0;
            0.15, 0, 0.00, 0
        ];
        q_limits = deg2rad([[-120 120]; [-110 90]; [-150 150]]);
    otherwise
        error('Unknown robot name: %s', name);
end
end

function poses = interpolate_pose_local(T0, T1, N)
poses = cell(N,1);
R0 = T0(1:3,1:3);
R1 = T1(1:3,1:3);
p0 = T0(1:3,4);
p1 = T1(1:3,4);
[axis, angle] = relative_axis_angle_local(R0, R1);
for k = 1:N
    s = (k-1)/(N-1);
    R = R0 * axang2rotm_fallback([axis.', angle * s]);
    p = (1-s)*p0 + s*p1;
    Tk = eye(4);
    Tk(1:3,1:3) = R;
    Tk(1:3,4) = p;
    poses{k} = Tk;
end
end

function q_traj = follow_path_local(dh_table, q0, waypoints, lambda)
if nargin < 4, lambda = 0.05; end
q = q0(:).';
q_traj = zeros(numel(waypoints), numel(q));
for k = 1:numel(waypoints)
    q = ik_to_pose_local(dh_table, q, waypoints{k}, lambda);
    q_traj(k,:) = q;
end
end

function q = ik_to_pose_local(dh_table, q, T_goal, lambda)
if nargin < 4, lambda = 0.05; end
max_iters = 30;
tol = 1e-4;
for iter = 1:max_iters
    [T, J] = fk_and_jacobian_local(dh_table, q);
    err = pose_error_local(T, T_goal);
    if norm(err) < tol
        return;
    end
    delta_q = dls_step_local(J, err, lambda);
    q = q + delta_q.';
end
warning('ik\_to\_pose fallback hit iteration cap with residual %.2e.', norm(err));
end

function [T, J] = fk_and_jacobian_local(dh_table, q)
Ts = eye(4);
J = zeros(6, numel(q));
origins = cell(numel(q)+1,1);
axes = cell(numel(q),1);
origins{1} = Ts(1:3,4);
for i = 1:numel(q)
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts = Ts * dh_local(a, alpha, d, theta);
    origins{i+1} = Ts(1:3,4);
    axes{i} = Ts(1:3,3);
end
T = Ts;
o_n = origins{end};
for i = 1:numel(q)
    J(1:3,i) = cross(axes{i}, o_n - origins{i});
    J(4:6,i) = axes{i};
end
end

function err = pose_error_local(T_current, T_target)
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

function delta_q = dls_step_local(J, err, lambda)
JT = J';
delta_q = JT * ((J * JT + (lambda^2) * eye(6)) \ err);
end

function J = jacobian_geometric_local(dh_table, q, ~)
[~, J] = fk_and_jacobian_local(dh_table, q);
end

function w = manipulability_local(J)
sigma = svd(J);
w = prod(sigma);
end

function [axis, angle] = relative_axis_angle_local(R0, R1)
R_rel = R0' * R1;
angle = acos(max(min((trace(R_rel) - 1)/2, 1), -1));
if angle < 1e-10
    axis = [0; 0; 1];
else
    axis = (1/(2*sin(angle))) * [R_rel(3,2) - R_rel(2,3);
                                  R_rel(1,3) - R_rel(3,1);
                                  R_rel(2,1) - R_rel(1,2)];
    axis = axis / norm(axis);
end
end

function T = transl(x, y, z)
T = eye(4);
T(1:3,4) = [x; y; z];
end

function T = rotZ(theta)
T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end

function T = dh_local(a, alpha, d, theta)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,             sin(alpha),             cos(alpha),            d;
              0,                     0,                     0,            1];
end

function R = axang2rotm_fallback(axang)
if exist('axang2rotm', 'file') || exist('axang2rotm', 'builtin')
    R = feval(@axang2rotm, axang);
else
    R = axang2rotm_local(axang);
end
end

function R = axang2rotm_local(axang)
axis = axang(1:3);
angle = axang(4);
axis = axis / norm(axis);
ux = axis(1); uy = axis(2); uz = axis(3);
c = cos(angle); s = sin(angle); v = 1 - c;
R = [ux*ux*v + c,    ux*uy*v - uz*s, ux*uz*v + uy*s;
     ux*uy*v + uz*s, uy*uy*v + c,    uy*uz*v - ux*s;
     ux*uz*v - uy*s, uy*uz*v + ux*s, uz*uz*v + c];
end
