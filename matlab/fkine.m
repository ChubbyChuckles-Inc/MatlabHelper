%% Lecture 4 — Forward kinematics utilities
% Step-by-step construction of FK helpers. Run each section to see the
% results populate in the Command Window.

clearvars;
clc;

%% 1. Build FK for a simple 3R manipulator
fprintf('\n=== Step 1: FK for a sample 3R joint configuration ===\n');
dh_table = [...
    0.3, 0, 0.1, 0;
    0.25, 0, 0.0, 0;
    0.15, 0, 0.0, 0
];
q = [20, -35, 50] * pi/180;
[T, Ts] = fkine_serial(dh_table, q, true);
tprint(T, 'End-effector pose');

%% 2. Extract pose components
fprintf('\n=== Step 2: Extract EE position and orientation ===\n');
position = ee_position(T);
orientation = ee_orientation_rpy(T);
fprintf('Position (m): [%.3f %.3f %.3f]\n', position);
fprintf('Orientation (deg): [roll=%.2f pitch=%.2f yaw=%.2f]\n', orientation * 180/pi);

%% Local functions

function [T, Ts] = fkine_serial(dh_table, q, verbose)
if nargin < 3, verbose = false; end
n = size(dh_table, 1);
Ts = cell(n+1, 1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ai = dh(a, alpha, d, theta);
    Ts{i+1} = Ts{i} * Ai;
    if verbose
        tprint(Ts{i+1}, sprintf('T_%d', i));
    end
end
T = Ts{end};
end

function pos = ee_position(T)
pos = T(1:3, 4).';
fprintf('ee_position → [%.3f %.3f %.3f] m\n', pos);
end

function rpy = ee_orientation_rpy(T)
R = T(1:3,1:3);
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
roll = atan2(R(3,2), R(3,3));
yaw = atan2(R(2,1), R(1,1));
rpy = [roll, pitch, yaw];
fprintf('ee_orientation_rpy → [%.2f %.2f %.2f] rad\n', rpy);
end

function A = dh(a, alpha, d, theta)
A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,             sin(alpha),             cos(alpha),            d;
              0,                     0,                     0,            1];
end

function tprint(T, label)
if nargin < 2, label = 'Transform'; end
fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f]\n', T(1:3,4));
fprintf('Rotation:\n');
disp(T(1:3,1:3));
end
