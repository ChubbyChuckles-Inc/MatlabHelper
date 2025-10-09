%% Lecture 8 — Pose interpolation helper
% Develop a pose interpolation routine by stepping through the theory.

clearvars;
clc;

%% 1. Define start/end poses
fprintf('\n=== Step 1: Define boundary poses ===\n');
T0 = transl(0.3, 0.1, 0.2) * rotZ(10*pi/180);
T1 = transl(0.6, -0.05, 0.35) * rotZ(80*pi/180) * rotY(15*pi/180);
tprint(T0, 'Start pose T0');
tprint(T1, 'Goal pose T1');

%% 2. Interpolate and inspect sample waypoints
fprintf('\n=== Step 2: Interpolate 6 waypoints ===\n');
poses = interpolate_pose(T0, T1, 6);
for k = 1:numel(poses)
    tprint(poses{k}, sprintf('Waypoint %d', k));
end

%% Local helper functions

function poses = interpolate_pose(T0, T1, N)
if N < 2, error('N must be ≥ 2.'); end
poses = cell(N,1);
R0 = T0(1:3,1:3);
R1 = T1(1:3,1:3);
p0 = T0(1:3,4);
p1 = T1(1:3,4);
[axis, angle] = relative_axis_angle(R0, R1);
for k = 1:N
    s = (k-1)/(N-1);
    p = (1-s)*p0 + s*p1;
    R = R0 * axang2rotm([axis.', angle * s]);
    Tk = eye(4);
    Tk(1:3,1:3) = R;
    Tk(1:3,4) = p;
    poses{k} = Tk;
    fprintf('interpolate_pose: waypoint %d/%d (s=%.2f)\n', k, N, s);
end
end

function [axis, angle] = relative_axis_angle(R0, R1)
R_rel = R0' * R1;
angle = acos(max(min((trace(R_rel) - 1)/2, 1), -1));
if abs(angle) < 1e-10
    axis = [0;0;1];
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
fprintf('transl → [%.3f %.3f %.3f]\n', x, y, z);
end

function T = rotZ(theta)
T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
fprintf('rotZ(%.2f°)\n', theta*180/pi);
end

function T = rotY(theta)
T = eye(4);
T([1 3],[1 3]) = [cos(theta), sin(theta); -sin(theta), cos(theta)];
fprintf('rotY(%.2f°)\n', theta*180/pi);
end

function tprint(T, label)
if nargin < 2, label = 'Transform'; end
fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f] m\n', T(1:3,4));
fprintf('Rotation:\n');
disp(T(1:3,1:3));
end
