%% Lecture 3 — Denavit–Hartenberg utilities
% This script develops reusable DH helpers. Run each section as you build
% understanding; verbose flags trigger Command Window prints to confirm the
% intermediate transforms.

clearvars;
clc;

%% 1. Single DH transform (standard convention)
fprintf('\n=== Step 1: Build an individual DH transform ===\n');
a = 0.35; alpha = 0; d = 0.1; theta = 25 * pi/180;
A1 = dh(a, alpha, d, theta, true);

%% 2. Chain multiple links and inspect frames
fprintf('\n=== Step 2: Chain DH links with verbose output ===\n');
dh_table = [...
    0.35, 0,  0.0,  30 * pi/180;
    0.25, 0,  0.0, -45 * pi/180
];
q = [30, -45] * pi/180;
[Ts, As] = chainDH(dh_table, q, true);
tprint(Ts{end}, 'End-effector pose from chainDH');

%% Local function definitions

function A = dh(a, alpha, d, theta, verbose)
if nargin < 5, verbose = false; end
A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,             sin(alpha),             cos(alpha),            d;
              0,                     0,                     0,            1];
if verbose
    fprintf('dh(a=%.3f, α=%.3f, d=%.3f, θ=%.3f rad) ->\n', a, alpha, d, theta);
    disp(A);
end
end

function [Ts, As] = chainDH(dh_table, q, verbose)
if nargin < 3, verbose = false; end
n = size(dh_table, 1);
Ts = cell(n+1, 1);
As = cell(n, 1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    As{i} = dh(a, alpha, d, theta, verbose);
    Ts{i+1} = Ts{i} * As{i};
    if verbose
        tprint(Ts{i+1}, sprintf('T_%d', i));
    end
end
end

function tprint(T, label)
if nargin < 2, label = 'Transform'; end
fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f] m\n', T(1:3,4));
fprintf('Rotation:\n');
disp(T(1:3,1:3));
end
