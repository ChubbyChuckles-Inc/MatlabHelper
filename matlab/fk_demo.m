%% Lecture 4 Demo — Forward kinematics exploration
% Execute sequential sections to observe FK behaviour in the Command Window
% (and a lightweight 3-D trajectory plot).

clearvars;
clc;

%% 1. Define DH parameters and a configuration sweep
fprintf('\n=== Step 1: Define manipulator and sweep q₁ ===\n');
dh_table = [...
    0.3, 0, 0.1, 0;
    0.25, 0, 0.0, 0;
    0.15, 0, 0.0, 0
];
q_nominal = [0, -30, 45] * pi/180;
[T_nom, ~] = fkine_serial(dh_table, q_nominal);
tprint(T_nom, 'Nominal pose');

%% 2. Sweep q₁ and log end-effector positions
fprintf('\n=== Step 2: Sweep q₁ from -60° to 60° ===\n');
q1_vals = linspace(-60, 60, 13) * pi/180;
traj = zeros(length(q1_vals), 3);
for k = 1:length(q1_vals)
    q = q_nominal;
    q(1) = q1_vals(k);
    [T, ~] = fkine_serial(dh_table, q);
    p = ee_position(T);
    traj(k,:) = p;
    fprintf('q₁=%6.1f° → position [%.3f %.3f %.3f] m\n', q1_vals(k)*180/pi, p);
end

%% 3. Minimal spatial visualisation
fprintf('\n=== Step 3: Plot sampled trajectory ===\n');
figure('Name', 'FK trajectory (q₁ sweep)');
plot3(traj(:,1), traj(:,2), traj(:,3), 'o-b', 'LineWidth', 1.5);
grid on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('End-effector trajectory as q₁ varies');
view(35, 25);

%% Local helper functions (duplicated here for self-containment)

function [T, Ts] = fkine_serial(dh_table, q)
n = size(dh_table, 1);
Ts = cell(n+1, 1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts{i+1} = Ts{i} * dh(a, alpha, d, theta);
end
T = Ts{end};
end

function pos = ee_position(T)
pos = T(1:3,4).';
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
fprintf('Position: [%.3f %.3f %.3f] m\n', T(1:3,4));
fprintf('Rotation:\n');
disp(T(1:3,1:3));
end
