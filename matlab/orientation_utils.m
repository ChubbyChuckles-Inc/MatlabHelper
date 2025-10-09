%% Lecture 2 — Orientation conversions: Euler (ZYX) vs. Axis-Angle
% Execute each section iteratively to see numerical results in the Command
% Window. Local helper functions reside at the end of the file and are
% instrumented with informative prints.

clearvars;
clc;

%% 1. Euler ZYX → rotation matrix, and back again
fprintf('\n=== Step 1: Euler ZYX conversions ===\n');
eulZYX = [30, -20, 10] * pi/180; % [yaw pitch roll]
R_euler = eulZYX2rotm(eulZYX);
recoveredZYX = rotm2eulZYX(R_euler);
fprintf('Recovered Euler ZYX (deg): [%.2f %.2f %.2f]\n', recoveredZYX * 180/pi);

%% 2. Axis-angle ↔ rotation matrix
fprintf('\n=== Step 2: Axis-Angle conversions ===\n');
axis = [1, 2, 0.5];
angle = 40 * pi/180;
R_axis = axang2rotm([axis, angle]);
[ux, uy, uz, ang_out] = rotm2axang(R_axis);
fprintf('Recovered axis-angle: [%.3f %.3f %.3f], %.2f deg\n', ux, uy, uz, ang_out * 180/pi);

%% 3. Compare Euler and axis-angle compositions
fprintf('\n=== Step 3: Compare composition methods ===\n');
R1 = eulZYX2rotm([45 20 -15] * pi/180);
R2 = axang2rotm([0 0 1, 30 * pi/180]) * axang2rotm([0 1 0, 20 * pi/180]);
comparison_error = max(abs(R1(:) - R2(:)));
fprintf('max(|R1 - R2|) = %.3e\n', comparison_error);

%% 4. Gimbal lock exploration (pitch → ±90°)
fprintf('\n=== Step 4: Gimbal lock exploration ===\n');
critical_pitch = 90 * pi/180;
R_gimbal = eulZYX2rotm([0, critical_pitch, 45 * pi/180]);
rotm2eulZYX(R_gimbal); % Prints gimbal lock notice

%% Local function definitions

function R = eulZYX2rotm(eulZYX)
yaw = eulZYX(1);
pitch = eulZYX(2);
roll = eulZYX(3);
Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
R = Rz * Ry * Rx;
fprintf('eulZYX2rotm: yaw=%.2f°, pitch=%.2f°, roll=%.2f°\n', yaw*180/pi, pitch*180/pi, roll*180/pi);
end

function eulZYX = rotm2eulZYX(R)
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
if abs(abs(pitch) - pi/2) < 1e-6
    fprintf('*** Gimbal lock detected at pitch ≈ %.2f°. ***\n', pitch * 180/pi);
    yaw = atan2(-R(1,2), R(2,2));
    roll = 0;
else
    yaw = atan2(R(2,1), R(1,1));
    roll = atan2(R(3,2), R(3,3));
end
eulZYX = [yaw, pitch, roll];
fprintf('rotm2eulZYX: yaw=%.2f°, pitch=%.2f°, roll=%.2f°\n', yaw*180/pi, pitch*180/pi, roll*180/pi);
end

function R = axang2rotm(axang)
axis = axang(1:3);
angle = axang(4);
[axis, was_normalized] = normalize_axis(axis);
ux = axis(1); uy = axis(2); uz = axis(3);
c = cos(angle); s = sin(angle); v = 1 - c;
R = [ux*ux*v + c,    ux*uy*v - uz*s, ux*uz*v + uy*s;
     ux*uy*v + uz*s, uy*uy*v + c,    uy*uz*v - ux*s;
     ux*uz*v - uy*s, uy*uz*v + ux*s, uz*uz*v + c];
if ~was_normalized
    warning('Axis was normalized internally before applying Rodrigues\' formula.');
end
fprintf('axang2rotm: axis=[%.3f %.3f %.3f], angle=%.2f°\n', axis, angle*180/pi);
end

function [ux, uy, uz, angle] = rotm2axang(R)
angle = acos(max(min((trace(R) - 1) / 2, 1), -1));
if abs(angle) < 1e-8
    ux = 1; uy = 0; uz = 0;
else
    ux = (R(3,2) - R(2,3)) / (2*sin(angle));
    uy = (R(1,3) - R(3,1)) / (2*sin(angle));
    uz = (R(2,1) - R(1,2)) / (2*sin(angle));
end
fprintf('rotm2axang: axis=[%.3f %.3f %.3f], angle=%.2f°\n', ux, uy, uz, angle*180/pi);
end

function [axis, was_normalized] = normalize_axis(axis)
norm_axis = norm(axis);
was_normalized = abs(norm_axis - 1) < 1e-9;
if norm_axis < 1e-12
    error('Axis vector is too small to normalize.');
end
axis = axis / norm_axis;
end
