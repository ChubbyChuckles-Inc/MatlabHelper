%% Lecture 4 — Practice Tasks on Forward Kinematics
% Prerequisite: add fkine_serial, ee_position, and ee_orientation_rpy from
% Lecture 4 to your MATLAB path. Tasks progress from evaluating a single
% configuration (Task 1) to exploring workspace reachability (Task 5).

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Evaluate FK for a 3R arm
% Goal: Use fkine_serial on a 3R planar arm and inspect the resulting pose.
fprintf('\n[Task 1] Step 1: Define the DH table and joint vector.\n');
dh_table = [
    0.35, 0, 0.05, 0;
    0.25, 0, 0.00, 0;
    0.20, 0, 0.00, 0
];
q = deg2rad([30, -40, 50]);
fprintf('[Task 1] Step 2: Compute FK with verbose output.\n');
[T, Ts] = fkine_serial(dh_table, q, true); %#ok<NASGU>
tprint(T, 'End-effector pose');

%% Task 2 — Extract point and orientation data
% Goal: Report the end-effector position and orientation using helper
% functions and double-check the numerical values.
fprintf('\n[Task 2] Step 1: Compute position and RPY.\n');
pos = ee_position(T);
orient = ee_orientation_rpy(T);
fprintf('Position (m): [%.3f %.3f %.3f]\n', pos);
fprintf('Orientation (deg): [%.2f %.2f %.2f]\n', rad2deg(orient));

%% Task 3 — Trace a joint sweep
% Goal: Sweep joint 2 from -60° to +60° (keeping others fixed) and store the
% resulting end-effector positions.
fprintf('\n[Task 3] Step 1: Sweep joint 2 and collect positions.\n');
q2_vals = deg2rad(linspace(-60, 60, 7));
path = zeros(numel(q2_vals), 3);
for i = 1:numel(q2_vals)
    q_sweep = [q(1), q2_vals(i), q(3)];
    T_i = fkine_serial(dh_table, q_sweep, false);
    path(i,:) = ee_position(T_i);
    fprintf('q2 = %+5.1f° → position = [%.3f %.3f %.3f] m\n', rad2deg(q2_vals(i)), path(i,:));
end

%% Task 4 — Compare orientations for mirrored joint sets
% Goal: Reflect joint 2 about zero (q2 → -q2) and quantify the change in task
% space orientation using ee_orientation_rpy.
fprintf('\n[Task 4] Step 1: Build mirrored configuration.\n');
q_mirror = [q(1), -q(2), q(3)];
T_mirror = fkine_serial(dh_table, q_mirror, false);
orient_mirror = ee_orientation_rpy(T_mirror);
orient_delta = rad2deg(orientediff(orient_mirror, orient));
fprintf('Mirrored orientation (deg): [%.2f %.2f %.2f]\n', rad2deg(orient_mirror));
fprintf('Delta orientation (deg): [%.2f %.2f %.2f]\n', orient_delta);

%% Task 5 — Coarse workspace search for target x-y reach
% Goal: Determine if the arm can place its end-effector within 5 mm of the
% task-space point [0.60, 0.10] m (ignoring z).
fprintf('\n[Task 5] Step 1: Create grid samples for q1 and q2.\n');
q1_samples = deg2rad(linspace(-60, 60, 13));
q2_samples = deg2rad(linspace(-90, 90, 19));
q3_fixed = q(3);
best_err = inf;
best_q = [];
for q1 = q1_samples
    for q2 = q2_samples
        T_candidate = fkine_serial(dh_table, [q1, q2, q3_fixed], false);
        pos_candidate = ee_position(T_candidate);
        err = norm(pos_candidate(1:2) - [0.60, 0.10]);
        if err < best_err
            best_err = err;
            best_q = [q1, q2];
        end
    end
end
fprintf('Best planar error = %.3e m at q = [%.1f°, %.1f°, %.1f°]\n', best_err, rad2deg(best_q(1)), rad2deg(best_q(2)), rad2deg(q3_fixed));

%% Helper for orientation difference (RPY subtraction in degrees)
function delta = orientediff(a, b)
% Wraps the difference between two RPY vectors into [-pi, pi].
raw = a - b;
delta = mod(raw + pi, 2*pi) - pi;
end
