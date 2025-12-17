% 2. Seminar - Denavit-Hartenberg, inverse und differentielle Kinematik
%
% Dieses Skript ist im gleichen Stil wie `seminar_1.m` aufgebaut und
% verwendet die dort definierten Hilfsfunktionen (rotX, rotY, rotZ,
% transl, rpy2rotm, rotm2rpy, tprint, visualize_transformation).
%
% Ziel: In ca. 5 Stunden durch die Themen
%   - Denavit-Hartenberg (DH) Konvention
%   - Inverse Kinematik
%   - Differentielle Kinematik
% zu führen. Der Fokus liegt auf Anschaulichkeit und Wiederverwendbarkeit
% der Funktionen.

clearvars;
clc;

% Sicherstellen, dass die Funktionen aus `seminar_1.m` verfügbar sind.
% Wir gehen davon aus, dass sich beide Skripte im gleichen Ordner
% `seminars` befinden. Der Ordner wird dem MATLAB-Pfad hinzugefügt und
% anschließend wird `seminar_1.m` einmal aufgerufen, um die dortigen
% Funktionsdefinitionen zu laden.
thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);
run(fullfile(thisDir, 'seminar_1.m'));

fprintf("=== 2. Seminar: DH, inverse und differentielle Kinematik ===\n");

%% 0. Voraussetzung: Basisfunktionen aus `seminar_1.m`
% Stelle sicher, dass `seminar_1.m` im MATLAB-Pfad liegt oder einmal
% ausgeführt wurde, sodass die Hilfsfunktionen im Workspace verfügbar sind.
% Alternativ können die Funktionsdefinitionen in eigene .m-Dateien
% ausgelagert und in den Pfad gelegt werden.

%% 1. Denavit-Hartenberg Konvention - Motivation und Grundidee
% In diesem Abschnitt definieren wir eine generische DH-Transformations-
% funktion und demonstrieren sie an einem 2-DOF-Planarroboter.

% DH-Transformationsmatrix nach Standard-DH:
%   a_i     : Länge der x_i-Achse (Abstand zwischen z_{i-1} und z_i)
%   alpha_i : Verdrehung um x_i (Winkel zwischen z_{i-1} und z_i)
%   d_i     : Verschiebung entlang z_{i-1}
%   theta_i : Verdrehung um z_{i-1}
%
% Diese Funktion kapselt die Standard-DH-Gleichung in eine wiederverwendbare
% Hilfsfunktion, damit wir im Seminar nur noch mit Parametern arbeiten und
% nicht jedes Mal die Matrix von Hand aufschreiben müssen.
function T = dh_transform(a, alpha, d, theta)
    % Trigonometrische Kurzschreibweisen zur besseren Lesbarkeit
    ca = cos(alpha); sa = sin(alpha);
    ct = cos(theta); st = sin(theta);

    % Standard-DH-Matrix: jede Zeile entspricht einer Kombination aus
    % Rotationen und Verschiebungen entlang/um z- bzw. x-Achse.
    T = [ ct, -st*ca,  st*sa, a*ct; ...
          st,  ct*ca, -ct*sa, a*st; ...
           0,     sa,     ca,    d; ...
           0,      0,      0,    1];
end

%% 1.1 Beispiel: Planarer 2R-Roboter im XY
fprintf('\n=== Abschnitt 1: DH-Konvention am 2R-Planarroboter ===\n');

% Gelenklängen (in m): einfache zweiarmige Kette im XY
L1 = 0.4;
L2 = 0.3;

% DH-Parameter für einen einfachen 2R-Planarroboter (Standard-DH):
% i | a_i  alpha_i  d_i  theta_i
% 1 | L1   0        0    q1
% 2 | L2   0        0    q2

q1 = deg2rad(30);
q2 = deg2rad(45);

T01 = dh_transform(L1, 0, 0, q1);
T12 = dh_transform(L2, 0, 0, q2);
T02 = T01 * T12;

tprint(T01, 'T_0^1 (2R-Planar)');
tprint(T02, 'T_0^2 (Endeffektor)');

% Visualisierung der drei Koordinatenrahmen (Basis, Frame 1, Frame 2)
fig_dh_2r = figure('Name', '2R-Planarroboter (DH)', 'NumberTitle', 'off');
ax_dh_2r = axes('Parent', fig_dh_2r);

% Basis -> Frame 1
visualize_transformation(T01, ax_dh_2r, 'Frame 0 zu 1');
% Basis -> Frame 2 direkt
visualize_transformation(T02, ax_dh_2r, 'Frame 0 zu 2');
hold(ax_dh_2r, 'on');
plot3(ax_dh_2r, [0, T01(1,4), T02(1,4)], [0, T01(2,4), T02(2,4)], [0, T01(3,4), T02(3,4)], 'k-', 'LineWidth', 2);
hold(ax_dh_2r, 'off');

%% 1.2 Interaktive Variation der Gelenkwinkel
% Idee für die Live-Demo: Variiere q1 und q2 und beobachte die Lage des
% Endeffektors. So kann man direkt sehen, wie die DH-Parameter die
% Endeffektorbahn im XY beeinflussen.

q1_vals = deg2rad(linspace(-90, 90, 7));
q2_vals = deg2rad(linspace(-90, 90, 7));

fprintf('\nBeispieltrajektorie für einige (q1,q2)-Kombinationen:\n');
for q1_val = q1_vals
    for q2_val = q2_vals
        T01 = dh_transform(L1, 0, 0, q1_val);
        T12 = dh_transform(L2, 0, 0, q2_val);
        T02 = T01 * T12;
        p = T02(1:3, 4);
        fprintf('q1 = %6.1f deg, q2 = %6.1f deg -> EE = [%.3f %.3f %.3f]\n', ...
            rad2deg(q1_val), rad2deg(q2_val), p(1), p(2), p(3));
    end
end

%% 2. Inverse Kinematik für den 2R-Planarroboter
% Wir bestimmen für eine gewünschte EE-Position (x,y) die Winkel q1, q2.

fprintf('\n=== Abschnitt 2: Inverse Kinematik 2R-Planar ===\n');

function [q1_solutions, q2_solutions] = ik_2r_planar(L1, L2, x, y)
    % Löst die inverse Kinematik für einen planaren 2R-Roboter im XY.
    r2 = x^2 + y^2;
    c2 = (r2 - L1^2 - L2^2) / (2*L1*L2);
    if abs(c2) > 1
        % |c2| > 1 bedeutet: der gewünschte Punkt liegt außerhalb des
        % erreichbaren Kreisanulus des 2R-Arms -> keine Lösung.
        error('ik_2r_planar:NoSolution', 'Zielpunkt außerhalb der Reichweite.');
    end
    s2_pos =  sqrt(1 - c2^2);
    s2_neg = -sqrt(1 - c2^2);

    q2_solutions = [atan2(s2_pos, c2), atan2(s2_neg, c2)];
    q1_solutions = zeros(1, 2);
    for i = 1:2
        q2 = q2_solutions(i);
        k1 = L1 + L2*cos(q2);
        k2 = L2*sin(q2);
        q1_solutions(i) = atan2(y, x) - atan2(k2, k1);
    end
end

x_target = 0.5;
y_target = 0.2;

[q1_s, q2_s] = ik_2r_planar(L1, L2, x_target, y_target);

for i = 1:numel(q1_s)
    % Jede IK-Lösung ist ein Paar (q1,q2), das den gleichen Zielpunkt
    % erreicht, aber ggf. eine andere Ellbogenkonfiguration hat
    % (Ellbogen oben/unten).
    fprintf('Lösung %d: q1 = %6.2f deg, q2 = %6.2f deg\n', i, rad2deg(q1_s(i)), rad2deg(q2_s(i)));
end

% Überprüfe die Vorwärtskinematik für beide Lösungen, um sicherzustellen,
% dass die IK-Berechnung konsistent zur FK ist.
for i = 1:numel(q1_s)
    T01 = dh_transform(L1, 0, 0, q1_s(i));
    T12 = dh_transform(L2, 0, 0, q2_s(i));
    T02 = T01 * T12;
    p = T02(1:3, 4);
    fprintf('FK(Lösung %d) -> [%.3f %.3f %.3f]\n', i, p(1), p(2), p(3));
end

%% 3. Differentielle Kinematik: Jacobi-Matrix für 2R-Planar
fprintf('\n=== Abschnitt 3: Differentielle Kinematik (Jacobi 2R) ===\n');

function J = jacobian_2r_planar(L1, L2, q1, q2)
    % Translationaler Jacobi für einen 2R-Planararm im XY.
    J11 = -L1*sin(q1) - L2*sin(q1+q2);
    J12 = -L2*sin(q1+q2);
    J21 =  L1*cos(q1) + L2*cos(q1+q2);
    J22 =  L2*cos(q1+q2);
    J = [J11, J12; J21, J22];
end

q1_demo = deg2rad(40);
q2_demo = deg2rad(-20);
J_demo = jacobian_2r_planar(L1, L2, q1_demo, q2_demo);

fprintf('Jacobi J(q) bei q = [%.1f, %.1f] deg:\n', rad2deg(q1_demo), rad2deg(q2_demo));

disp(J_demo);

% Interpretiere J als Abbildung von Gelenkgeschwindigkeiten zu
% Kartesischen Geschwindigkeiten: v = J * q_dot.
qdot = [0.1; 0.2];  % rad/s
vxy  = J_demo * qdot;
fprintf('Kartesische Geschwindigkeit v = [vx vy]^T = [%.4f %.4f] m/s\n', vxy(1), vxy(2));

%% 4. Erweiterung auf 3D: 3-achsiger Arm mit DH und Jacobi
fprintf('\n=== Abschnitt 4: 3D-Roboter mit DH und Jacobi ===\n');

% Beispiel: Ein einfacher 3DOF-Arm (R-R-R), z.B. Schulter-Schulter-Ellbogen
L1_3D = 0.3;
L2_3D = 0.2;
L3_3D = 0.15;

% DH-Parameter (vereinfachtes Beispiel):
% i | a_i      alpha_i         d_i    theta_i
% 1 | 0        +pi/2           0      q1
% 2 | L2_3D    0               0      q2
% 3 | L3_3D    0               0      q3

function T = fk_3d_rrr(q, L2_3D, L3_3D)
    q1 = q(1); q2 = q(2); q3 = q(3);
    T01 = dh_transform(0,  pi/2, 0, q1);
    T12 = dh_transform(L2_3D, 0, 0, q2);
    T23 = dh_transform(L3_3D, 0, 0, q3);
    T03 = T01 * T12 * T23;
    T = T03;
end

q_3d = deg2rad([20, 30, -10]);
T03_demo = fk_3d_rrr(q_3d, L2_3D, L3_3D);

tprint(T03_demo, 'T_0^3 (3DOF-RRR)');

% Visualisierung der Transformation Basis -> EE im 3D-Raum.
fig_3d = figure('Name', '3DOF-RRR (DH)', 'NumberTitle', 'off');
ax_3d = axes('Parent', fig_3d);
visualize_transformation(T03_demo, ax_3d, 'Basis zu EE (3DOF-RRR)');

%% 4.1 Differentielle Kinematik: numerische Approximation der Jacobi
% Wir berechnen die Jacobi-Matrix für den 3DOF-Arm numerisch über
% kleine Störungen der Gelenkwinkel.

function J = numerical_jacobian_fk3d(fk_handle, q, delta)
    if nargin < 3
        delta = 1e-6;
    end
    n = numel(q);
    J = zeros(3, n);
    T0 = fk_handle(q);
    p0 = T0(1:3, 4);
    for i = 1:n
        dq = zeros(size(q));
        dq(i) = delta;
        T_pert = fk_handle(q + dq);
        p_pert = T_pert(1:3, 4);
        J(:, i) = (p_pert - p0) / delta;
    end
end

J_3d = numerical_jacobian_fk3d(@(qq) fk_3d_rrr(qq, L2_3D, L3_3D), q_3d, 1e-6);

fprintf('Numerische Jacobi für 3DOF-RRR bei q = [%.1f %.1f %.1f] deg:\n', ...
    q_3d(1)*180/pi, q_3d(2)*180/pi, q_3d(3)*180/pi);

disp(J_3d);

%% 5. Anwendungsaufgaben

%% Aufgabe 1: Workspace-Analyse 2R-Planar
% a) Schreibe eine Schleife, die für q1 in [-90°,90°] und q2 in
%    [-90°,90°] den Endeffektor-Standort berechnet und in einem 2D-Plot
%    darstellt.
% b) Markiere den Bereich, der mehrfach erreicht werden kann
%    (Redundanz / Mehrfachlösungen).
% Musterlösung (a): einfache Rasterung und 2D-Plot des Arbeitsraums
figure('Name', 'Workspace 2R-Planar', 'NumberTitle', 'off');
ax_ws = axes; hold(ax_ws, 'on'); grid(ax_ws, 'on'); axis(ax_ws, 'equal');
title(ax_ws, 'Arbeitsraum 2R-Planar (x-y-Ebene)'); xlabel(ax_ws, 'x [m]'); ylabel(ax_ws, 'y [m]');

q1_grid = deg2rad(linspace(-90, 90, 91));
q2_grid = deg2rad(linspace(-90, 90, 91));

workspace_points = [];
for q1_tmp = q1_grid
    for q2_tmp = q2_grid
        T01_tmp = dh_transform(L1, 0, 0, q1_tmp);
        T12_tmp = dh_transform(L2, 0, 0, q2_tmp);
        T02_tmp = T01_tmp * T12_tmp;
        p_tmp = T02_tmp(1:3, 4);
        workspace_points = [workspace_points; p_tmp(1:2)']; %#ok<AGROW>
    end
end
scatter(ax_ws, workspace_points(:,1), workspace_points(:,2), 5, 'b', 'filled', 'DisplayName', 'erreichbare Punkte');
legend(ax_ws, 'Location', 'best');

% Hinweis zu (b): Mehrfachlösungen erkennt man daran, dass unterschiedliche
% (q1,q2)-Paare auf den gleichen (x,y)-Punkt abbilden. Dies lässt sich in
% einer Übung z.B. mit einem Distanzschwellwert zwischen Punkten prüfen.

%% Aufgabe 2: Inverse Kinematik als Funktion
% Implementiere eine Funktion `ik_2r_planar_all`, die für eine Liste von
% Zielpunkten [(x1, y1), (x2, y2), ...] jeweils die (bis zu zwei)
% möglichen Gelenkkonfigurationen berechnet und zurückgibt.
% Teste deine Funktion mit mindestens 5 beliebigen Punkten und vergleiche
% die Vorwärtskinematik mit den ursprünglichen Zielpunkten.
% Musterlösung: Hilfsfunktion, die IK für mehrere Punkte auswertet
function solutions = ik_2r_planar_all(L1_local, L2_local, xy_targets)
    % xy_targets ist eine N×2-Matrix mit [x_i, y_i]
    n_pts = size(xy_targets, 1);
    solutions = cell(n_pts, 1);  % jede Zelle enthält bis zu zwei (q1,q2)-Paare
    for k = 1:n_pts
        xk = xy_targets(k, 1);
        yk = xy_targets(k, 2);
        try
            [q1_sol, q2_sol] = ik_2r_planar(L1_local, L2_local, xk, yk);
            solutions{k} = [q1_sol(:), q2_sol(:)];
        catch ME
            % Falls keine Lösung existiert, speichere eine leere Matrix
            warning('Punkt (%.3f, %.3f) nicht erreichbar: %s', xk, yk, ME.message);
            solutions{k} = [];
        end
    end
end

% Beispielaufruf der Musterlösung für fünf Zielpunkte
targets_example = [0.4 0.1; 0.5 0.2; 0.3 0.2; 0.2 0.4; 0.1 0.1];
all_solutions = ik_2r_planar_all(L1, L2, targets_example);

for k = 1:size(targets_example,1)
    xy = targets_example(k,:);
    sols = all_solutions{k};
    fprintf('Zielpunkt %d: (x,y) = [%.3f %.3f]\n', k, xy(1), xy(2));
    if isempty(sols)
        fprintf('  -> keine IK-Lösung im Modellbereich\n');
    else
        for s = 1:size(sols,1)
            q1k = sols(s,1); q2k = sols(s,2);
            T01_k = dh_transform(L1, 0, 0, q1k);
            T12_k = dh_transform(L2, 0, 0, q2k);
            T02_k = T01_k * T12_k;
            p_k = T02_k(1:3, 4);
            fprintf('  Lösung %d: q1 = %6.2f deg, q2 = %6.2f deg -> FK = [%.3f %.3f %.3f]\n', ...
                s, rad2deg(q1k), rad2deg(q2k), p_k(1), p_k(2), p_k(3));
        end
    end
end

%% Aufgabe 3: Trajektorienplanung im Gelenkraum
% Plane eine zeitabhängige Trajektorie q(t) für den 2R-Roboter, z.B.
% als S-Kurven- oder lineares Profil von einer Startkonfiguration zur
% Zielkonfiguration. Nutze deine Jacobi-Funktion, um die resultierenden
% kartesischen Geschwindigkeiten v(t) über der Zeit zu plotten.
% Musterlösung: einfache lineare Gelenkraumtrajektorie und zugehörige
% kartesische Geschwindigkeiten über Jacobi
q_start = deg2rad([0;  0]);      % Startkonfiguration
q_goal  = deg2rad([60; 30]);     % Zielkonfiguration
T_total = 2.0;                   % Gesamtdauer in Sekunden
dt = 0.01;
t_vec = 0:dt:T_total;

q_traj   = zeros(2, numel(t_vec));
qdot_traj = zeros(2, numel(t_vec));
v_traj   = zeros(2, numel(t_vec));

for idx = 1:numel(t_vec)
    tau = t_vec(idx) / T_total;  % normierte Zeit in [0,1]
    % Lineare Interpolation: q(t) = q_start + tau * (q_goal - q_start)
    q_traj(:, idx) = q_start + tau * (q_goal - q_start);
    % Konstante Gelenkgeschwindigkeit für lineare Interpolation
    qdot_traj(:, idx) = (q_goal - q_start) / T_total;

    q1_i = q_traj(1, idx); q2_i = q_traj(2, idx);
    J_i = jacobian_2r_planar(L1, L2, q1_i, q2_i);
    v_traj(:, idx) = J_i * qdot_traj(:, idx);  % v = J * q_dot
end

figure('Name', 'Kartesische Geschwindigkeiten v_x, v_y über der Zeit', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t_vec, v_traj(1,:), 'LineWidth', 1.5); grid on;
ylabel('v_x [m/s]');
title('Kartesische Geschwindigkeiten des EE über der Zeit');
subplot(2,1,2);
plot(t_vec, v_traj(2,:), 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('v_y [m/s]');

%% Aufgabe 4: Orientierungserweiterung für den 3DOF-Arm
% Erweitere die Funktion `fk_3d_rrr` so, dass nicht nur die Position,
% sondern auch eine sinnvoll gewählte Orientierung des Endeffektors als
% Roll-Pitch-Yaw-Winkel zurückgegeben wird. Nutze `rpy2rotm` und
% `rotm2rpy` aus `seminar_1.m`, um Konsistenz zu prüfen.
% Musterlösung: erweiterte FK-Funktion, die zusätzlich RPY-Winkel liefert
function [T_ee, rpy_ee] = fk_3d_rrr_with_rpy(q, L2_3D_local, L3_3D_local)
    % Nutze die bestehende fk_3d_rrr-Funktion für die Position/Rotation
    T_ee = fk_3d_rrr(q, L2_3D_local, L3_3D_local);
    % Extrahiere RPY-Winkel aus der Rotationsmatrix
    rpy_ee = rotm2rpy(T_ee);
end

% Beispielaufruf und Konsistenzprüfung mit rpy2rotm
q_test = deg2rad([10, 20, 30]);
[T_ee_test, rpy_ee_test] = fk_3d_rrr_with_rpy(q_test, L2_3D, L3_3D);

fprintf('\nErweiterte FK 3DOF-RRR: RPY-Winkel (rad) = [%.3f %.3f %.3f]\n', ...
    rpy_ee_test(1), rpy_ee_test(2), rpy_ee_test(3));

% Rekonstruiere Rotationsmatrix aus den berechneten RPY-Winkeln
T_from_rpy = rpy2rotm(rpy_ee_test);
fprintf('Abweichung zwischen FK-Rotationsmatrix und rpy2rotm(RPY): \n');
disp(T_from_rpy(1:3,1:3) - T_ee_test(1:3,1:3));

%% Aufgabe 5: Vergleich numerische vs. analytische Jacobi
% Leite (falls möglich) eine analytische Jacobi-Matrix für den
% 3DOF-RRR-Arm her und implementiere sie als Funktion. Vergleiche die
% Ergebnisse mit der numerisch berechneten Jacobi für mindestens 10
% Zufallskonfigurationen im zulässigen Gelenkwinkelbereich.
% Musterlösung (Skizze): numerische vs. analytische Jacobi vergleichen.
% Hier wird nur die Vergleichsstruktur gezeigt – die eigentliche
% analytische Jacobi-Funktion `jacobian_3d_rrr_analytic` muss im Rahmen
% der Übung hergeleitet und implementiert werden.

% Beispiel einer möglichen Signatur:
% function J_analytic = jacobian_3d_rrr_analytic(q, L2_3D_local, L3_3D_local)
%     % ... analytische Herleitung einfügen ...
% end

% Vergleichsschleife für 10 Zufallskonfigurationen (Pseudocode):
% for k = 1:10
%     q_rand = deg2rad([-90 + 180*rand, -90 + 180*rand, -90 + 180*rand]);
%     J_num = numerical_jacobian_fk3d(@(qq) fk_3d_rrr(qq, L2_3D, L3_3D), q_rand, 1e-6);
%     J_an  = jacobian_3d_rrr_analytic(q_rand, L2_3D, L3_3D);
%     fprintf('Konfiguration %d: ||J_num - J_an||_F = %.3e\n', k, norm(J_num - J_an, 'fro'));
% end
