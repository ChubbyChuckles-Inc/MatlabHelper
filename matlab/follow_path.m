%% Lecture 8 — Follow a path via iterative IK
% Step-by-step construction of a path follower that marches through task-
% space waypoints while reporting progress.

clearvars;
clc;

%% 1. Load a simple 3R robot model
fprintf('\n=== Step 1: Load DH model ===\n');
[dh_table, q_limits] = load_robot_dh('3R_planar');
disp(dh_table);

%% 2. Define waypoints and follow the path
fprintf('\n=== Step 2: Follow path of interpolated poses ===\n');
T0 = transl(0.3, 0.0, 0.1);
T1 = transl(0.55, 0.1, 0.15) * rotZ(45*pi/180);
waypoints = interpolate_pose(T0, T1, 5);
q0 = [0, -20, 35] * pi/180;
trajectory = follow_path(dh_table, q0, waypoints, 0.05);
fprintf('Completed %d waypoints.\n', size(trajectory,1));

%% Local helper functions

function q_traj = follow_path(dh_table, q0, waypoints, lambda)
if nargin < 4, lambda = 0.05; end
max_iters = 20;
tol = 1e-4;
q = q0(:).';
q_traj = zeros(numel(waypoints), numel(q));
for k = 1:numel(waypoints)
    T_goal = waypoints{k};
    fprintf('Waypoint %d/%d\n', k, numel(waypoints));
    for iter = 1:max_iters
        [T_current, J] = fk_and_jacobian(dh_table, q);
        err = pose_error(T_current, T_goal);
        if norm(err) < tol
            fprintf('  Converged in %d iterations (residual %.2e).\n', iter-1, norm(err));
            break;
        end
        delta_q = dls_step(J, err, lambda);
        q = q + delta_q.';
        fprintf('  Iter %02d: ‖err‖=%.2e, ‖Δq‖=%.2e\n', iter, norm(err), norm(delta_q));
    end
    q_traj(k,:) = q;
end
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
    axis = axis / max(norm(axis), eps);
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
fprintf('Loaded robot ''%s'' with %d joints.\n', name, size(dh_table,1));
end

function T = transl(x, y, z)
T = eye(4);
T(1:3,4) = [x; y; z];
end

function T = rotZ(theta)
T = eye(4);
T(1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end
