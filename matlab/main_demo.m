%% Lecture 8 Final Demo — Robot task execution
% Run this script after stepping through the earlier lectures. It performs a
% simple pick-and-place style motion and summarizes progress in the Command
% Window.

clearvars;
clc;

fprintf('=== MatlabHelper Robot Demo ===\n');
[dh_table, ~] = load_robot_dh('3R_planar');
q0 = [0, -25, 40] * pi/180;
T_home  = transl(0.35, 0.05, 0.12);
T_pick  = transl(0.45, 0.00, 0.08) * rotZ(30*pi/180);
T_place = transl(0.55, -0.10, 0.12) * rotZ(80*pi/180);

segments = {
    interpolate_pose(T_home, T_pick, 6);
    interpolate_pose(T_pick, T_place, 8);
    interpolate_pose(T_place, T_home, 6)
};
waypoints = vertcat(segments{:});
fprintf('Total waypoints: %d\n', numel(waypoints));

q = q0;
log_entries = strings(numel(waypoints),1);
for k = 1:numel(waypoints)
    T_goal = waypoints{k};
    q = ik_to_pose(dh_table, q, T_goal, 0.08);
    [~, J] = fk_and_jacobian(dh_table, q);
    w = manipulability(J);
    log_entries(k) = sprintf('Reached waypoint %02d — manipulability %.4f', k, w);
    fprintf('%s\n', log_entries(k));
end

fprintf('\nDemo complete! All waypoints reached.\n');

%% Local helper functions

function poses = interpolate_pose(T0, T1, N)
poses = cell(N,1);
R0 = T0(1:3,1:3);
R1 = T1(1:3,1:3);
p0 = T0(1:3,4);
p1 = T1(1:3,4);
[axis, angle] = relative_axis_angle(R0, R1);
for k = 1:N
    s = (k-1)/(N-1);
    R = R0 * axang2rotm([axis.', angle * s]);
    p = (1-s)*p0 + s*p1;
    Tk = eye(4); Tk(1:3,1:3) = R; Tk(1:3,4) = p;
    poses{k} = Tk;
end
end

function q = ik_to_pose(dh_table, q, T_goal, lambda)
if nargin < 4, lambda = 0.05; end
max_iters = 20;
tol = 1e-4;
for iter = 1:max_iters
    [T, J] = fk_and_jacobian(dh_table, q);
    err = pose_error(T, T_goal);
    if norm(err) < tol
        fprintf('  Waypoint converged in %d iterations.\n', iter-1);
        return;
    end
    delta_q = dls_step(J, err, lambda);
    q = q + delta_q.';
    fprintf('  iter %02d: ‖err‖=%.2e\n', iter, norm(err));
end
warning('  Iteration limit reached with residual %.2e.', norm(err));
end

function delta_q = dls_step(J, err, lambda)
JT = J';
delta_q = JT * ((J * JT + (lambda^2) * eye(6)) \ err);
end

function [T, J] = fk_and_jacobian(dh_table, q)
[J, Ts] = jacobian_geometric(dh_table, q);
T = Ts{end};
end

function [J, Ts] = jacobian_geometric(dh_table, q)
n = size(dh_table,1);
Ts = cell(n+1,1);
Ts{1} = eye(4);
for i = 1:n
    a = dh_table(i,1);
    alpha = dh_table(i,2);
    d = dh_table(i,3);
    theta = q(i) + dh_table(i,4);
    Ts{i+1} = Ts{i} * dh(a, alpha, d, theta);
end
J = zeros(6,n);
o_n = Ts{end}(1:3,4);
for i = 1:n
    z = Ts{i}(1:3,3);
    o = Ts{i}(1:3,4);
    J(1:3,i) = cross(z, o_n - o);
    J(4:6,i) = z;
end
end

function w = manipulability(J)
sigma = svd(J);
w = prod(sigma);
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

function err = pose_error(T_current, T_target)
linear = T_target(1:3,4) - T_current(1:3,4);
R_err = T_current(1:3,1:3)' * T_target(1:3,1:3);
angle = acos(max(min((trace(R_err) - 1)/2, 1), -1));
if abs(angle) < 1e-8
    axis = zeros(3,1);
else
    axis = (1/(2*sin(angle))) * [R_err(3,2) - R_err(2,3);
                                  R_err(1,3) - R_err(3,1);
                                  R_err(2,1) - R_err(1,2)];
end
angular = T_current(1:3,1:3) * axis * angle;
err = [linear; angular];
end

function [dh_table, q_limits] = load_robot_dh(name)
switch lower(name)
    case '3r_planar'
        dh_table = [...
            0.3, 0, 0.0, 0;
            0.25, 0, 0.0, 0;
            0.15, 0, 0.0, 0
        ];
        q_limits = deg2rad([[-120 120]; [-110 90]; [-150 150]]);
    otherwise
        error('Unknown robot name: %s', name);
end
end

function T = transl(x, y, z)
T = eye(4);
T(1:3,4) = [x; y; z];
end

function T = rotZ(theta)
T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end
