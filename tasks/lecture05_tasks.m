%% Lecture 5 — Practice Tasks on Planar 2R Inverse Kinematics
% Prerequisite: add ikine_2R and validate_ik_2R from Lecture 5 to the MATLAB
% path. Tasks cover branch comparison, reachability analysis, and workspace
% exploration, culminating in a mini pick-and-place plan (Task 5).

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Solve IK for a single reachable point
% Goal: Use ikine_2R to reach (0.42, 0.18) m with link lengths 0.35 m and
% 0.25 m; confirm feasibility and residual.
fprintf('\n[Task 1] Step 1: Solve for elbow-down branch.\n');
[q_down, feasible_down] = ikine_2R(0.42, 0.18, 0.35, 0.25, 'down');
fprintf('Elbow-down solution (deg): [%.2f %.2f], feasible = %d\n', rad2deg(q_down), feasible_down);
validate_ik_2R(0.42, 0.18, 0.35, 0.25, 'down');

%% Task 2 — Contrast elbow-up vs elbow-down
% Goal: Compute both branches for the same point and compare the end-effector
% locations via forward kinematics.
fprintf('\n[Task 2] Step 1: Solve for the elbow-up branch.\n');
[q_up, feasible_up] = ikine_2R(0.42, 0.18, 0.35, 0.25, 'up');
fprintf('Elbow-up solution (deg): [%.2f %.2f], feasible = %d\n', rad2deg(q_up), feasible_up);
fprintf('[Task 2] Step 2: Compare FK results.\n');
p_down = planar_fk(q_down, 0.35, 0.25);
p_up = planar_fk(q_up, 0.35, 0.25);
fprintf('Elbow-down FK: [%.3f %.3f] m\n', p_down);
fprintf('Elbow-up FK:   [%.3f %.3f] m\n', p_up);

%% Task 3 — Enumerate reachability over a grid
% Goal: Sample points on a polar grid and record which are reachable.
fprintf('\n[Task 3] Step 1: Create a polar lattice.\n');
r_values = linspace(0.05, 0.60, 8);
theta_values = linspace(-pi/2, pi/2, 13);
reachable = zeros(numel(r_values), numel(theta_values));
for i = 1:numel(r_values)
    for j = 1:numel(theta_values)
        x = r_values(i) * cos(theta_values(j));
        y = r_values(i) * sin(theta_values(j));
        [~, feasible] = ikine_2R(x, y, 0.35, 0.25, 'down');
        reachable(i,j) = feasible;
    end
end
fprintf('Number of reachable samples: %d/%d\n', nnz(reachable), numel(reachable));

%% Task 4 — Track joint movement along a Cartesian line
% Goal: Displace the target from x=0.30 m to x=0.55 m at y=0.15 m and chart
% how q1, q2 evolve for the elbow-down branch.
fprintf('\n[Task 4] Step 1: Sweep x along the line and solve IK.\n');
x_line = linspace(0.30, 0.55, 9);
traj = zeros(numel(x_line), 2);
for idx = 1:numel(x_line)
    [q_line, feasible] = ikine_2R(x_line(idx), 0.15, 0.35, 0.25, 'down');
    if feasible
        traj(idx,:) = q_line;
        fprintf('x = %.3f → q = [%.1f°, %.1f°]\n', x_line(idx), rad2deg(q_line));
    else
        fprintf('x = %.3f is unreachable.\n', x_line(idx));
    end
end

%% Task 5 — Mini pick-and-place sequence
% Goal: Given pick point (0.38, 0.10), hover (0.38, 0.18), drop (0.28, 0.12),
% compute joint targets for each waypoint and confirm feasibility.
fprintf('\n[Task 5] Step 1: Define the waypoints.\n');
waypoints = [
    0.38, 0.10;
    0.38, 0.18;
    0.28, 0.12
];
branches = {'down', 'down', 'up'}; % intentionally mix branches for practice
solutions = zeros(size(waypoints));
for k = 1:size(waypoints,1)
    [q_task, feasible] = ikine_2R(waypoints(k,1), waypoints(k,2), 0.35, 0.25, branches{k});
    if feasible
        solutions(k,:) = q_task;
        fprintf('Waypoint %d (%s) → q = [%.1f°, %.1f°]\n', k, branches{k}, rad2deg(q_task));
    else
        fprintf('Waypoint %d is unreachable with branch %s.\n', k, branches{k});
    end
end
fprintf('[Task 5] Step 2: Validate residuals.\n');
for k = 1:size(waypoints,1)
    validate_ik_2R(waypoints(k,1), waypoints(k,2), 0.35, 0.25, branches{k});
end

%% Local helper mirroring the lecture derivation
function p = planar_fk(q, l1, l2)
% Compute planar FK for a 2R manipulator.
x = l1 * cos(q(1)) + l2 * cos(q(1) + q(2));
y = l1 * sin(q(1)) + l2 * sin(q(1) + q(2));
p = [x; y];
end
