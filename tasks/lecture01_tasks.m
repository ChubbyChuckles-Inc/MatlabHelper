%% Lecture 1 — Practice Tasks on Frames and Primitives
% Prerequisite: ensure the helper functions you authored during Lecture 1
% (rotX, rotY, rotZ, transl, rpy2rotm, rotm2rpy, tprint) are on the MATLAB
% path. Each task is structured from an easy warm-up (Task 1) to a more
% involved challenge (Task 5). Run the sections one-by-one after confirming
% your function implementations are available.

taskDir = fileparts(mfilename('fullpath'));
addpath(fullfile(taskDir, '..', 'matlab'));

%% Task 1 — Compose a simple rotation about Z
% Goal: Use rotZ to rotate the world x-axis by +30° and confirm the rotated
% vector matches the expected orientation.
fprintf('\n[Task 1] Step 1: Create the transform using rotZ.\n');
Rz = rotZ(deg2rad(30));
tprint(Rz, 'Rotation about +Z by 30°');
fprintf('[Task 1] Step 2: Rotate the unit x-axis.\n');
ex = [1; 0; 0; 1];
ex_rotated = Rz * ex;
fprintf('Rotated x-axis homogeneous vector: [%.3f %.3f %.3f %.0f]^T\n', ex_rotated);

%% Task 2 — Move a point with translation then rotation
% Goal: Translate a point by (0.2, -0.1, 0.05) m and apply a -20° rotation
% about Y. Report both intermediate and final positions.
fprintf('\n[Task 2] Step 1: Build the translation.\n');
T_translate = transl(0.2, -0.1, 0.05);
tprint(T_translate, 'Pure translation');
fprintf('[Task 2] Step 2: Apply the rotation about Y.\n');
Ty = rotY(deg2rad(-20));
T_total = T_translate * Ty;
tprint(T_total, 'Combined transform');
fprintf('[Task 2] Step 3: Transform point p = [0.1 0.0 0.05 1]^T.\n');
p = [0.1; 0.0; 0.05; 1];
p_world = T_total * p;
fprintf('Transformed point: [%.3f %.3f %.3f]^T m\n', p_world(1:3));

%% Task 3 — Validate RPY ↔ rotation conversions
% Goal: Convert an arbitrary RPY triplet to a rotation matrix and recover the
% angles. Report the reconstruction error in degrees.
fprintf('\n[Task 3] Step 1: Define the test roll/pitch/yaw.\n');
rpy_test = deg2rad([15, 25, -10]);
fprintf('Original RPY (deg): [%.2f %.2f %.2f]\n', rad2deg(rpy_test));
fprintf('[Task 3] Step 2: Convert to a rotation matrix.\n');
R = rpy2rotm(rpy_test);
tprint(R, 'Rotation from RPY');
fprintf('[Task 3] Step 3: Recover the angles with rotm2rpy.\n');
rpy_recovered = rotm2rpy(R);
err = rad2deg(rpy_recovered - rpy_test);
fprintf('Recovered RPY (deg): [%.2f %.2f %.2f]\n', rad2deg(rpy_recovered));
fprintf('Reconstruction error (deg): ‖Δ‖₂ = %.3e\n', norm(err));

%% Task 4 — Check SO(3) quality for chained rotations
% Goal: Chain rotX, rotY, rotZ in sequence and evaluate orthogonality and
% determinant to confirm the result lies in SO(3).
fprintf('\n[Task 4] Step 1: Compose chained rotations.\n');
chain = rotX(deg2rad(40)) * rotY(deg2rad(-15)) * rotZ(deg2rad(35));
tprint(chain, 'Chained rotation');
fprintf('[Task 4] Step 2: Extract and test the 3×3 block.\n');
R3 = chain(1:3, 1:3);
orthogonality_error = norm(R3' * R3 - eye(3));
determinant = det(R3);
fprintf('‖R^T R - I‖₂ = %.3e\n', orthogonality_error);
fprintf('det(R) = %.6f\n', determinant);

%% Task 5 — Align a tool frame to a desired RPY target
% Goal: Starting from the identity transform, apply translations and rotations
% so the end frame matches a given (position, RPY) target. Measure the final
% position and orientation error to confirm alignment.
fprintf('\n[Task 5] Step 1: Specify the desired pose.\n');
p_des = [0.25, -0.05, 0.12];
rpy_des = deg2rad([20, -30, 15]);
T_des = transl(p_des(1), p_des(2), p_des(3)) * rpy2rotm(rpy_des);
tprint(T_des, 'Desired pose');
fprintf('[Task 5] Step 2: Construct the pose step-by-step.\n');
T_built = eye(4);
T_built = T_built * transl(p_des(1), 0, 0);
T_built = T_built * transl(0, p_des(2), 0);
T_built = T_built * transl(0, 0, p_des(3));
T_built = T_built * rotZ(rpy_des(3));
T_built = T_built * rotY(rpy_des(2));
T_built = T_built * rotX(rpy_des(1));
tprint(T_built, 'Constructed pose');
fprintf('[Task 5] Step 3: Quantify the misalignment.\n');
pos_error = norm(T_built(1:3,4) - p_des');
R_error = T_built(1:3,1:3)' * T_des(1:3,1:3);
angle_error = acos(max(min((trace(R_error) - 1)/2, 1), -1));
fprintf('Position error: %.3e m\n', pos_error);
fprintf('Orientation error: %.3e rad\n', angle_error);
