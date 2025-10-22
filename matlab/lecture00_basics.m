function lecture00_basics()
% Lecture 0 — MATLAB essentials for robotics (math, visualization, kinematics)
%
% Purpose
%   This overview orients you to MATLAB's math and plotting capabilities,
%   then bridges into core robotics kinematics concepts. It is designed so
%   that exporting the Command Window and figures to PDF provides a compact
%   study reference. Run this file top-to-bottom (sections are ordered for
%   clarity). All helper functions are defined BEFORE they are used.
%
% How to run
%   - Open this file in MATLAB and press Run (or run section by section with %% cells)
%   - Ensure the current folder is the repository root or the 'matlab' folder
%   - Figures are created inline; keep them open when exporting to PDF
%
% What you'll practice
%   1) MATLAB math essentials (arrays, linear algebra, numerics)
%   2) Visualization: 2D, 3D, surfaces, vector fields, coordinate frames
%   3) Kinematics: rotation matrices, homogeneous transforms, simple chains
%   4) DH-based FK (brief), with frame drawing utilities
%
% Conventions
%   - Angles are in radians unless explicitly printed in degrees
%   - Right-handed coordinate frames; Z out of the page by default
%   - Transform T maps from a source frame to a target frame (post-multiply)
%   - R is 3x3 rotation, p is 3x1 position, T is 4x4 homogeneous transform

%% Helper functions (defined first, then used below)

    function out = d2r(deg)
        out = deg * pi/180;
    end

    function out = r2d(rad)
        out = rad * 180/pi;
    end

    function R = rotX3(theta)
        c = cos(theta); s = sin(theta);
        R = [1 0 0; 0 c -s; 0 s c];
    end

    function R = rotY3(theta)
        c = cos(theta); s = sin(theta);
        R = [c 0 s; 0 1 0; -s 0 c];
    end

    function R = rotZ3(theta)
        c = cos(theta); s = sin(theta);
        R = [c -s 0; s c 0; 0 0 1];
    end

    function T = makeT(R, p)
        T = eye(4);
        T(1:3,1:3) = R;
        T(1:3,4) = p(:);
    end

    function T = transl4(x, y, z)
        T = eye(4);
        T(1:3,4) = [x; y; z];
    end

    function tf = isSO3(R, tol)
        if nargin < 2, tol = 1e-9; end
        tf = norm(R'*R - eye(3), 2) < tol && abs(det(R) - 1) < tol;
    end

    function tprint4(T, label)
        if nargin < 2, label = 'Transform'; end
        fprintf('\n%s\n', label);
        fprintf('  p (m): [%.4f %.4f %.4f]\n', T(1,4), T(2,4), T(3,4));
        fprintf('  R =\n');
        disp(T(1:3,1:3));
    end

    function draw_frame3(ax, T, scale, lw)
        % Draw a coordinate frame at transform T into axes ax
        if nargin < 3 || isempty(scale), scale = 0.1; end
        if nargin < 4 || isempty(lw), lw = 2; end
        o = T(1:3,4);
        x = T(1:3,1) * scale;
        y = T(1:3,2) * scale;
        z = T(1:3,3) * scale;
        hold(ax, 'on');
        quiver3(ax, o(1), o(2), o(3), x(1), x(2), x(3), 0, 'r', 'LineWidth', lw, 'MaxHeadSize', 0.5);
        quiver3(ax, o(1), o(2), o(3), y(1), y(2), y(3), 0, 'g', 'LineWidth', lw, 'MaxHeadSize', 0.5);
        quiver3(ax, o(1), o(2), o(3), z(1), z(2), z(3), 0, 'b', 'LineWidth', lw, 'MaxHeadSize', 0.5);
    end

    function A = dh(a, alpha, d, theta)
        c = cos(theta); s = sin(theta); ca = cos(alpha); sa = sin(alpha);
        A = [c, -s*ca,  s*sa, a*c;
             s,  c*ca, -c*sa, a*s;
             0,     sa,    ca,   d;
             0,      0,     0,   1];
    end

    function [Tn, Ts] = fkine_dh(dh_table, q)
        n = size(dh_table,1);
        Ts = cell(n+1,1); Ts{1} = eye(4);
        for i=1:n
            a = dh_table(i,1); alpha = dh_table(i,2); d = dh_table(i,3); th0 = dh_table(i,4);
            th = q(i) + th0;
            Ts{i+1} = Ts{i} * dh(a, alpha, d, th);
        end
        Tn = Ts{end};
    end

    function plot_planar_chain(ax, points, opt)
        if nargin < 3, opt = struct(); end
        if ~isfield(opt,'lw'), opt.lw = 3; end
        if ~isfield(opt,'ms'), opt.ms = 30; end
        plot(ax, points(1,:), points(2,:), '-o', 'LineWidth', opt.lw, 'MarkerSize', opt.ms/3);
        axis(ax, 'equal'); grid(ax, 'on');
        xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
    end

%% 1) Getting started: housekeeping
clc; rng(0);
fprintf('Lecture 0: MATLAB essentials for robotics\n');
fprintf('Date: %s\n', datestr(now));

%% 2) Math essentials: arrays and linear algebra
fprintf('\n=== Math essentials ===\n');

% Arrays and element-wise math
v = (1:5);                   % row vector
w = v.^2;                    % element-wise power
A = [1 2; 3 4];              % 2x2 matrix
B = [2 -1; 1 0.5];
C = A*B;                     % matrix-matrix product
D = A.*B;                    % element-wise multiply
fprintf('A*B =\n'); disp(C);
fprintf('A.*B =\n'); disp(D);

% Indexing and slicing
M = reshape(1:12, [3,4]);    % 3x4 matrix with values 1..12
fprintf('M(2,3) = %d, M(:,2) = [ %s ]\n', M(2,3), num2str(M(:,2).'));

% Implicit expansion (broadcasting)
row = [1 2 3]; col = [10; 20; 30];
S = row + col;               % 3x3 by broadcasting
fprintf('row + col =\n'); disp(S);

% Solving linear systems and conditioning
A = [4 1 0; 1 3 -1; 0 -1 2]; b = [1; 2; 0.5];
x = A\b; res = norm(A*x - b);
fprintf('Solve Ax=b with \\: ‖Ax-b‖ = %.2e\n', res);
fprintf('cond(A)2 = %.2f\n', cond(A));

% SVD and least squares
M = randn(5,3); y = randn(5,1);
[U,S,V] = svd(M, 'econ'); x_ls = M\y; sig = diag(S).';
fprintf('SVD(M): singular values = [%.3f %.3f %.3f]\n', sig);
fprintf('Least-squares solution norm = %.3f\n', norm(x_ls));

%% 3) Visualization: 2D and 3D
fprintf('\n=== Visualization basics ===\n');

% 2D plotting
t = linspace(0, 2*pi, 200);
y1 = sin(t); y2 = cos(t);
figure('Name','2D plots');
subplot(1,2,1);
plot(t, y1, 'b-', 'LineWidth', 1.5); hold on; plot(t, y2, 'r--', 'LineWidth', 1.5);
grid on; xlabel('t (rad)'); ylabel('amplitude'); legend('sin(t)', 'cos(t)'); title('Sine and Cosine');

% Scatter with labels
xdata = linspace(-1,1,20); ydata = xdata.^2 + 0.05*randn(size(xdata));
subplot(1,2,2);
scatter(xdata, ydata, 40, 'filled'); grid on; xlabel('x'); ylabel('y'); title('Scatter example');

% 3D plotting
figure('Name','3D plots');
subplot(1,2,1);
t = linspace(0, 4*pi, 200); r = 0.2; z = linspace(0, 1, 200);
x = r*cos(t); y = r*sin(t);
plot3(x, y, z, 'm-', 'LineWidth', 2); grid on; axis equal; xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D helix');

subplot(1,2,2);
[X,Y] = meshgrid(linspace(-1,1,50)); Z = exp(-3*(X.^2 + Y.^2));
surf(X,Y,Z); shading interp; colormap turbo; axis tight;
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Gaussian bump (surf)');

%% 4) Rotations and homogeneous transforms
fprintf('\n=== Rotations and transforms ===\n');
R = rotZ3(d2r(30)) * rotY3(d2r(15)) * rotX3(d2r(-10));
fprintf('isSO3(R) = %d, det(R) = %.6f\n', isSO3(R), det(R));
p = [0.3; -0.1; 0.2]; T = makeT(R, p);
tprint4(T, 'Example transform T');

% Compose transforms: base->A->B
Ta = makeT(rotZ3(d2r(20)), [0.2; 0.0; 0.0]);
Tb = makeT(rotY3(d2r(10)), [0.0; 0.1; 0.0]);
T_ab = Ta * Tb; tprint4(T_ab, 'Composed T = Ta * Tb');

%% 5) Visualizing coordinate frames in 3D
fprintf('\n=== Frame visualization ===\n');
fig = figure('Name','Frames'); ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 45, 25);
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z'); title(ax, 'Base and rotated frames');
draw_frame3(ax, eye(4), 0.1, 2);          % base frame
draw_frame3(ax, T, 0.1, 2);               % transformed frame
legend(ax, {'X','Y','Z'});                % color meaning standard: X-red, Y-green, Z-blue

%% 6) Planar 2R chain demo (no DH)
fprintf('\n=== Planar 2R forward kinematics (geometry) ===\n');
l1 = 0.35; l2 = 0.25; q = [d2r(35), d2r(-25)];
p0 = [0;0];
p1 = p0 + [l1*cos(q(1)); l1*sin(q(1))];
p2 = p1 + [l2*cos(q(1)+q(2)); l2*sin(q(1)+q(2))];
fprintf('End-effector (x,y) = [%.3f, %.3f] m\n', p2(1), p2(2));

fig2 = figure('Name','Planar 2R'); ax2 = axes(fig2);
plot_planar_chain(ax2, [p0 p1 p2]); title(ax2, '2R planar arm');
text(ax2, p2(1), p2(2), sprintf('  EE [%.2f, %.2f] m', p2(1), p2(2)));

%% 7) DH-based FK (brief) and frame drawing
fprintf('\n=== DH-based forward kinematics ===\n');
dh_table = [
    l1, 0, 0, 0;  % a, alpha, d, theta_offset
    l2, 0, 0, 0
];
[Tee, Ts] = fkine_dh(dh_table, q);
tprint4(Tee, 'T_{base->ee} from DH chain');

% Draw intermediate frames
fig3 = figure('Name','DH Frames'); ax3 = axes(fig3);
hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal'); view(ax3, 30, 25);
xlabel(ax3, 'X'); ylabel(ax3, 'Y'); zlabel(ax3, 'Z'); title(ax3, 'Frames along DH chain');
for i = 1:numel(Ts)
    draw_frame3(ax3, Ts{i}, 0.08, 2);
end
draw_frame3(ax3, Tee, 0.1, 3);

fprintf('\nEnd of Lecture 0. Suggested next step: run Lecture 1 utilities.\n');
end
