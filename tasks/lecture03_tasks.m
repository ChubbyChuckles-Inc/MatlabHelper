%% Lecture 3 — Practice Tasks on DH Parameters
% Prerequisite: add your DH helpers (dh, chainDH, tprint) from Lecture 3 to
% the MATLAB path. Each task builds from a single-link review (Task 1) up to
% reasoning about pose targets for a 3-link chain (Task 5).

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Review a single DH transform
% Goal: Compute the homogeneous matrix for the parameters a=0.4, α=30°, d=0.1,
% θ=45° and inspect the resulting frame.
fprintf('\n[Task 1] Step 1: Build the transform with dh().\n');
A1 = dh(0.4, deg2rad(30), 0.1, deg2rad(45), true);
tprint(A1, 'DH transform for link 1');

%% Task 2 — Chain a pair of revolute joints
% Goal: Use chainDH to form a 2R planar arm and evaluate the end-effector
% pose for q = [25°, -35°].
fprintf('\n[Task 2] Step 1: Define the DH table.\n');
dh_table = [
    0.35, 0, 0.0, 0;
    0.25, 0, 0.0, 0
];
q = deg2rad([25, -35]);
fprintf('[Task 2] Step 2: Call chainDH with verbose output.\n');
[Ts, As] = chainDH(dh_table, q, true); %#ok<NASGU>
tprint(Ts{end}, 'End-effector pose');

%% Task 3 — Trace intermediate frames
% Goal: Record the origins of each intermediate frame for the DH table above
% and compute the distances between successive frames.
fprintf('\n[Task 3] Step 1: Extract frame origins.\n');
origins = cellfun(@(T) T(1:3,4), Ts, 'UniformOutput', false);
for i = 1:numel(origins)
    fprintf('Origin of frame %d: [%.3f %.3f %.3f] m\n', i-1, origins{i});
end
fprintf('[Task 3] Step 2: Compute link offsets.\n');
for i = 1:numel(origins)-1
    d = norm(origins{i+1} - origins{i});
    fprintf('Distance between frame %d and %d: %.3f m\n', i-1, i, d);
end

%% Task 4 — Explore joint offsets
% Goal: Introduce a 10° joint offset on joint 2 and observe how the EE pose
% changes relative to the nominal configuration.
fprintf('\n[Task 4] Step 1: Apply joint offset in the DH table.\n');
dh_offset = dh_table;
dh_offset(2,4) = deg2rad(10);
[Ts_offset, ~] = chainDH(dh_offset, q, false);
T_nominal = Ts{end};
T_offset = Ts_offset{end};
fprintf('[Task 4] Step 2: Compare EE positions.\n');
pos_delta = T_offset(1:3,4) - T_nominal(1:3,4);
fprintf('Position change: [%.3f %.3f %.3f] m\n', pos_delta);

%% Task 5 — Solve for a joint angle given a target x-position
% Goal: For the same 2R arm, determine the second joint angle q2 that places
% the end-effector at x = 0.50 m when q1 = 20°.
fprintf('\n[Task 5] Step 1: Define search parameters.\n');
q1 = deg2rad(20);
x_target = 0.50;
fprintf('[Task 5] Step 2: Sample q2 over a grid and evaluate the x-position.\n');
q2_grid = deg2rad(linspace(-120, 120, 241));
x_values = zeros(size(q2_grid));
for idx = 1:numel(q2_grid)
    [Ts_candidate, ~] = chainDH(dh_table, [q1, q2_grid(idx)], false);
    x_values(idx) = Ts_candidate{end}(1,4);
end
[~, best_idx] = min(abs(x_values - x_target));
q2_solution = q2_grid(best_idx);
fprintf('Best q2 ≈ %.2f° (x=%.3f m)\n', rad2deg(q2_solution), x_values(best_idx));
