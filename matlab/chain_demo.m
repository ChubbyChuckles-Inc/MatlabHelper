%% Lecture 3 Demo — Chaining DH transforms for a 2R planar arm
% Execute the sections sequentially to visualise intermediate frames and
% final end-effector positions.

clearvars;
clc;
addpath(fileparts(mfilename('fullpath')));

%% 1. Define DH parameters for a planar 2R arm
fprintf('\n=== Step 1: Define link parameters ===\n');
dh_table = [...
    0.4, 0, 0, 0;  % link 1
    0.3, 0, 0, 0   % link 2
];
q = [45, -30] * pi/180;
[Ts, ~] = chainDH(dh_table, q, true);
tprint(Ts{end}, 'End-effector pose at q=[45°,-30°]');

%% 2. Sweep zero configuration to verify reachability along x-axis
fprintf('\n=== Step 2: Zero configuration reachability ===\n');
q_zero = [0, 0];
Ts_zero = chainDH(dh_table, q_zero, false);
position_zero = Ts_zero{end}(1:3,4);
fprintf('EE position at zero configuration: [%.3f %.3f %.3f] m\n', position_zero);

%% 3. Sample additional postures and list positions
fprintf('\n=== Step 3: Sampled postures ===\n');
configs_deg = [0 0; 30 30; 60 -45; -45 60];
for idx = 1:size(configs_deg,1)
    q_sample = configs_deg(idx,:) * pi/180;
    Ts_sample = chainDH(dh_table, q_sample, false);
    p = Ts_sample{end}(1:3,4);
    fprintf('q=[%5.1f %5.1f]° → position [%.3f %.3f %.3f] m\n', configs_deg(idx,1), configs_deg(idx,2), p);
end
