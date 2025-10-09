%% Lecture 2 — Practice Tasks on Orientation Representations
% Prerequisite: add the Lecture 2 helpers (eulZYX2rotm, rotm2eulZYX,
% axang2rotm, rotm2axang) to your MATLAB path. Each section ramps up in
% difficulty from conceptual checks (Task 1) to multi-representation
% reasoning (Task 5).

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Euler ZYX round-trip sanity check
% Goal: Convert a ZYX Euler triple to a rotation matrix and immediately back.
fprintf('\n[Task 1] Step 1: Define yaw/pitch/roll.\n');
eulZYX = deg2rad([40, -10, 25]);
fprintf('Original ZYX angles (deg): [%.2f %.2f %.2f]\n', rad2deg(eulZYX));
fprintf('[Task 1] Step 2: Convert to rotation matrix and back.\n');
R = eulZYX2rotm(eulZYX);
eul_rec = rotm2eulZYX(R);
fprintf('Recovered ZYX angles (deg): [%.2f %.2f %.2f]\n', rad2deg(eul_rec));
fprintf('Angular error (deg): %.3e\n', norm(rad2deg(eul_rec - eulZYX)));

%% Task 2 — Axis-angle synthesis of a known rotation
% Goal: Form a 90° rotation about the (1,1,0) axis and confirm the axis and
% angle estimates returned by rotm2axang.
fprintf('\n[Task 2] Step 1: Build the rotation using axis-angle.\n');
axis = [1, 1, 0];
R_axis = axang2rotm([axis, deg2rad(90)]);
fprintf('[Task 2] Step 2: Recover axis and angle.\n');
[ux, uy, uz, ang] = rotm2axang(R_axis);
fprintf('Recovered axis: [%.3f %.3f %.3f], angle = %.2f°\n', ux, uy, uz, rad2deg(ang));

%% Task 3 — Compare Euler and axis-angle routes to the same pose
% Goal: Create a target orientation using Euler angles and verify the result
% matches an axis-angle construction to within numerical tolerance.
fprintf('\n[Task 3] Step 1: Euler construction.\n');
R_euler = eulZYX2rotm(deg2rad([20, 35, -15]));
fprintf('[Task 3] Step 2: Axis-angle construction.\n');
axis2 = [0.5, 0.2, 0.8];
angle2 = deg2rad(48);
R_axis2 = axang2rotm([axis2, angle2]);
fprintf('[Task 3] Step 3: Blend axis-angle to match Euler by solving for delta.\n');
axis_delta = [0.2, 0.6, 0.1];
angle_delta = deg2rad(-12);
R_adjusted = R_axis2 * axang2rotm([axis_delta, angle_delta]);
err = max(abs(R_euler(:) - R_adjusted(:)));
fprintf('Max element-wise difference: %.3e\n', err);

%% Task 4 — Investigate gimbal lock near pitch = ±90°
% Goal: Create a rotation with pitch ≈ 90° and inspect the fallback strategy
% of rotm2eulZYX when the system is near gimbal lock.
fprintf('\n[Task 4] Step 1: Build a nearly singular orientation.\n');
eul_lock = deg2rad([10, 89.9, 45]);
R_lock = eulZYX2rotm(eul_lock);
fprintf('[Task 4] Step 2: Recover angles and observe printed warning.\n');
rotm2eulZYX(R_lock);

%% Task 5 — Blend multiple representations for a trajectory
% Goal: Given two orientations expressed in mixed forms, create an in-between
% pose and express it in both Euler ZYX and axis-angle coordinates.
fprintf('\n[Task 5] Step 1: Build the start/end orientations.\n');
R_start = eulZYX2rotm(deg2rad([0, 20, -10]));
R_end = axang2rotm([0, 0, 1, deg2rad(60)]) * axang2rotm([0, 1, 0, deg2rad(15)]);
fprintf('[Task 5] Step 2: Interpolate halfway via axis-angle delta.\n');
R_delta = R_start' * R_end;
[ux_mid, uy_mid, uz_mid, ang_mid] = rotm2axang(R_delta);
R_mid = R_start * axang2rotm([ux_mid, uy_mid, uz_mid, ang_mid/2]);
fprintf('[Task 5] Step 3: Report the midpoint in both parameterizations.\n');
eul_mid = rotm2eulZYX(R_mid);
[ax_mid_x, ax_mid_y, ax_mid_z, ang_mid_total] = rotm2axang(R_mid);
fprintf('Midpoint Euler ZYX (deg): [%.2f %.2f %.2f]\n', rad2deg(eul_mid));
fprintf('Midpoint axis-angle: axis=[%.3f %.3f %.3f], angle=%.2f°\n', ax_mid_x, ax_mid_y, ax_mid_z, rad2deg(ang_mid_total));
