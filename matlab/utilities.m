%% Lecture 1 — MATLAB scripting essentials and reference frames
% This script is structured in sections so you can run each block from the
% MATLAB Script Editor and immediately observe the results in the Command
% Window. Local function definitions live at the end of the file so the
% narrative flows from theory to practice.

clearvars;
clc;

%% 1. Build basic rotation and translation primitives
fprintf('\n=== Step 1: Construct primitive transforms ===\n');
rx = rotX(pi/6);
ry = rotY(pi/12);
rz = rotZ(-pi/8);
tprint(rx, 'Rotation about X (30 deg)');
tprint(ry, 'Rotation about Y (15 deg)');
tprint(rz, 'Rotation about Z (-22.5 deg)');

%% 2. Compose a pose using homogeneous matrices
fprintf('\n=== Step 2: Compose a base-to-EE transform ===\n');
T = transl(0.3, 0.1, 0.2) * rotZ(pi/6) * rotY(pi/12);
tprint(T, 'Base→EE');

%% 3. Convert between roll-pitch-yaw and rotation matrices
fprintf('\n=== Step 3: RPY conversions ===\n');
rpy = [10, 20, 5] * pi/180; % [roll pitch yaw] in radians
tprint(rpy2rotm(rpy), 'RPY rotation');
recovered = rotm2rpy(rpy2rotm(rpy));
fprintf('Recovered RPY (deg): [%.2f %.2f %.2f]\n', recovered * 180/pi);

%% 4. Check orthonormality and determinant conditions
fprintf('\n=== Step 4: Rotation quality checks ===\n');
R_full = rotZ(pi/6) * rotY(pi/12);
R = R_full(1:3,1:3);
orthogonality_error = norm(R' * R - eye(3));
determinant = det(R);
fprintf('‖R^T R − I‖₂ = %.3e\n', orthogonality_error);
fprintf('det(R)         = %.6f\n', determinant);
if orthogonality_error < 1e-10 && abs(determinant - 1) < 1e-10
    fprintf('Rotation matrix passes SO(3) checks.\n');
else
    warning('Rotation matrix failed orthogonality/determinant test.');
end

%% Local function definitions
% These helpers return values AND print useful context so students receive
% instant feedback while building them interactively.

function T = rotX(theta)
T = eye(4);
T(2:3,2:3) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
fprintf('rotX(%.3f rad) created.\n', theta);
end

function T = rotY(theta)
T = eye(4);
T([1 3],[1 3]) = [cos(theta), sin(theta); -sin(theta), cos(theta)];
fprintf('rotY(%.3f rad) created.\n', theta);
end

function T = rotZ(theta)
T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
fprintf('rotZ(%.3f rad) created.\n', theta);
end

function T = transl(x, y, z)
T = eye(4);
T(1:3,4) = [x; y; z];
fprintf('transl([%.3f %.3f %.3f]) created.\n', x, y, z);
end

function T = rpy2rotm(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
T = rotZ(yaw) * rotY(pitch) * rotX(roll);
fprintf('rpy2rotm -> yaw=%.3f, pitch=%.3f, roll=%.3f rad.\n', yaw, pitch, roll);
end

function rpy = rotm2rpy(T)
R = T(1:3,1:3);
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
if abs(cos(pitch)) < 1e-8
    warning('Gimbal lock encountered: using fallback for roll/yaw.');
    roll = atan2(R(1,2), R(2,2));
    yaw = 0;
else
    roll = atan2(R(3,2), R(3,3));
    yaw  = atan2(R(2,1), R(1,1));
end
rpy = [roll, pitch, yaw];
fprintf('rotm2rpy -> roll=%.3f, pitch=%.3f, yaw=%.3f rad.\n', roll, pitch, yaw);
end

function tprint(T, label)
if nargin < 2, label = 'Transform'; end
fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f] m\n', T(1:3,4));
fprintf('Rotation (top-left 3×3):\n');
disp(T(1:3,1:3));
end
