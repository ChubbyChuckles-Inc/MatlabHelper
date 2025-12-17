% 3. Seminar - Singularitäten in der Roboterkinematik
%
% Dieses Seminar ist wie `seminar_1.m` und `seminar_2.m` aufgebaut:
% - viele Zwischenüberschriften (%%)
% - ausführliche, erklärende Kommentare (auch zu wichtigen Codezeilen)
% - wiederverwendbare Hilfsfunktionen
% - anschauliche Plots und Visualisierungen
%
% Thema: Singularitäten in der Roboterkinematik
% Dauer: ca. 3 Stunden
%
% Inhalte:
%   1) Was bedeutet "Singularität" im Kontext der Jacobi-Matrix?
%   2) Planarer 2R-Roboter: det(J), Rangverlust und anschauliche Beispiele
%   3) Konditionszahl, Manipulierbarkeit und Geschwindigkeits-Ellipse
%   4) 3D-Serienkette: geometrische Jacobi-Matrix und Rang
%   5) Warum IK nahe Singularitäten instabil wird (Pseudoinverse) und wie
%      Damped Least Squares (DLS) hilft
%   6) Anwendungsaufgaben

clearvars;
clc;

% Hinweis zur Wiederverwendung (wichtig für die Aufgabenstellung):
% Dieses Seminar nutzt ausschließlich
%   - MATLAB-Bordmittel (Built-ins)
%   - sowie Funktionsdefinitionen, die in `seminar_1.m` und `seminar_2.m`
%     im Rahmen der vorherigen Vorführungen entwickelt wurden.
%
% Da MATLAB "lokale Funktionen" in Skriptdateien nicht automatisch für
% andere Skripte exportiert, sind die benötigten Funktionen aus Seminar 1
% und 2 hier am Dateiende als lokale Funktionen eingebettet.

fprintf("=== 3. Seminar: Singularitäten in der Roboterkinematik ===\n");

%% 1. Grundidee: Singularität = Rangverlust der Jacobi
% Eine Singularität ist (vereinfacht) eine Konfiguration, in der die
% Jacobi-Matrix J Rang verliert.
%
% Intuition:
% - J bildet Gelenkgeschwindigkeiten q_dot auf Endeffektor-Geschwindigkeiten v ab.
% - Wenn J Rang verliert, dann gibt es Richtungen im Arbeitsraum, die man
%   mit keiner endlichen Gelenkgeschwindigkeit mehr erzeugen kann.
% - Umgekehrt kann es sein, dass kleine Endeffektorbewegungen "sehr große"
%   Gelenkbewegungen erfordern (numerisch instabil -> IK-Probleme).
%
% Wir starten mit dem einfachsten und anschaulichsten Beispiel: 2R planar.

%% 2. Beispiel 1: Planarer 2R-Roboter und seine Singularitäten
fprintf('\n=== Abschnitt 2: 2R-Planar – Singularität bei gestrecktem Arm ===\n');

% Roboterparameter (Längen in m)
L1 = 0.40;
L2 = 0.30;

% Wichtiger Fakt (analytisch):
% Für den 2R-Planarroboter ist det(J) = L1 * L2 * sin(q2).
% -> Singular bei q2 = 0° oder q2 = 180° (Arm gestreckt oder komplett gefaltet).

% Wir wählen eine feste q1 und variieren q2.
q1 = deg2rad(20);
q2_samples = deg2rad(linspace(-170, 170, 171));

detJ = zeros(size(q2_samples));
condJ = zeros(size(q2_samples));
manip = zeros(size(q2_samples));

for k = 1:numel(q2_samples)
    q2 = q2_samples(k);
    J = jacobian_2r_planar(L1, L2, q1, q2);     % 2x2-Jacobi im XY
    detJ(k) = det(J);
    condJ(k) = cond(J);                         % Konditionszahl (-> groß nahe Singularität)
    % Yoshikawa-Manipulierbarkeit (ohne Zusatzfunktion): Produkt der
    % Singularwerte von J. In der Singularität wird mindestens eine
    % Singularzahl ~ 0 -> w ~ 0.
    s = svd(J);
    manip(k) = prod(s);
end

% Plot 1: det(J) über q2
figure('Name', '2R: det(J) und Konditionszahl', 'NumberTitle', 'off');
subplot(3,1,1);
plot(rad2deg(q2_samples), detJ, 'LineWidth', 1.5);
grid on;
ylabel('det(J)');
title('2R-Planar: det(J), Konditionszahl und Manipulierbarkeit über q2');

% Plot 2: Konditionszahl (log, weil sie sehr groß werden kann)
subplot(3,1,2);
semilogy(rad2deg(q2_samples), condJ, 'LineWidth', 1.5);
grid on;
ylabel('cond(J)');

% Plot 3: Manipulierbarkeit
subplot(3,1,3);
plot(rad2deg(q2_samples), manip, 'LineWidth', 1.5);
grid on;
xlabel('q2 [deg]');
ylabel('w');

%% 2.1 Visualisierung: gestreckt vs. nicht-gestreckt
% Wir visualisieren zwei Konfigurationen im XY:
% - q2 ≈ 0° (singulär)
% - q2 ≈ 60° (nicht singulär)

q_sing = [q1; deg2rad(0)];
q_ok   = [q1; deg2rad(60)];

figure('Name', '2R-Planar: Geometrie nahe Singularität', 'NumberTitle', 'off');
ax_geom = axes; hold(ax_geom, 'on'); grid(ax_geom, 'on'); axis(ax_geom, 'equal');
xlabel(ax_geom, 'x [m]'); ylabel(ax_geom, 'y [m]');
title(ax_geom, '2R-Planar: Gestreckt (singulär) vs. gebeugt (nicht singulär)');

% Zeichne den Arm als Linienzug: p0 -> p1 -> p2
% (MATLAB-Bordmittel, keine neue Hilfsfunktion)
q1s = q_sing(1); q2s = q_sing(2);
p0 = [0; 0];
p1s = [L1*cos(q1s); L1*sin(q1s)];
p2s = p1s + [L2*cos(q1s+q2s); L2*sin(q1s+q2s)];
plot(ax_geom, [p0(1) p1s(1) p2s(1)], [p0(2) p1s(2) p2s(2)], 'r-', 'LineWidth', 2, 'DisplayName', 'Singulär (q2=0°)');
scatter(ax_geom, [p0(1) p1s(1) p2s(1)], [p0(2) p1s(2) p2s(2)], 40, 'filled', 'HandleVisibility', 'off');

q1o = q_ok(1); q2o = q_ok(2);
p1o = [L1*cos(q1o); L1*sin(q1o)];
p2o = p1o + [L2*cos(q1o+q2o); L2*sin(q1o+q2o)];
plot(ax_geom, [p0(1) p1o(1) p2o(1)], [p0(2) p1o(2) p2o(2)], 'b-', 'LineWidth', 2, 'DisplayName', 'Nicht singulär (q2=60°)');
scatter(ax_geom, [p0(1) p1o(1) p2o(1)], [p0(2) p1o(2) p2o(2)], 40, 'filled', 'HandleVisibility', 'off');

legend(ax_geom, 'Location', 'best');

%% 3. Geschwindigkeitsabbildung: Geschwindigkeits-Ellipse (2D)
% In 2D kann man die Wirkung von J sehr anschaulich zeigen:
%
% Idee:
% - Wir betrachten alle Gelenkgeschwindigkeiten q_dot mit ||q_dot|| = 1.
%   Das ist im Gelenkraum ein Einheitskreis.
% - Die resultierenden Endeffektor-Geschwindigkeiten sind v = J * q_dot.
% - Das Bild eines Kreises unter einer linearen Abbildung ist eine Ellipse.
% - In der Singularität kollabiert diese Ellipse zu einer Linie (oder einem Punkt).

fprintf('\n=== Abschnitt 3: Geschwindigkeits-Ellipse und Singularität ===\n');

figure('Name', '2R: Geschwindigkeits-Ellipse', 'NumberTitle', 'off');
ax_ell = axes; hold(ax_ell, 'on'); grid(ax_ell, 'on'); axis(ax_ell, 'equal');
xlabel(ax_ell, 'v_x (skal.)'); ylabel(ax_ell, 'v_y (skal.)');
title(ax_ell, 'Bild des Einheitskreises unter v = J(q) q̇');

J_sing = jacobian_2r_planar(L1, L2, q_sing(1), q_sing(2));
J_ok   = jacobian_2r_planar(L1, L2, q_ok(1),   q_ok(2));

% Wir sampeln den Einheitskreis im Gelenkraum und plotten v = J*q_dot.
N = 200;
theta = linspace(0, 2*pi, N);
qd = [cos(theta); sin(theta)];

v_sing = J_sing * qd;
v_ok   = J_ok   * qd;

plot(ax_ell, v_sing(1,:), v_sing(2,:), 'r', 'LineWidth', 2, 'DisplayName', 'Singulär');
plot(ax_ell, v_ok(1,:),   v_ok(2,:),   'b', 'LineWidth', 2, 'DisplayName', 'Nicht singulär');
legend(ax_ell, 'Location', 'best');

%% 4. Beispiel 2: 3D-Serienkette und Rangverlust einer translatorischen Jacobi
fprintf('\n=== Abschnitt 4: 3D-Kette – Rang, Singulärwerte und Manipulierbarkeit ===\n');

% Wir nehmen den 3DOF-RRR-Arm aus `seminar_2.m` und approximieren seine
% translatorische Jacobi numerisch (nur Position, 3x3).

L2_3D = 0.20;
L3_3D = 0.15;

q_straight = deg2rad([0, 0, 0]);
q_bent     = deg2rad([10, -40, 30]);

T_straight = fk_3d_rrr(q_straight, L2_3D, L3_3D);
T_bent     = fk_3d_rrr(q_bent,     L2_3D, L3_3D);

Jv_straight = numerical_jacobian_fk3d(@(qq) fk_3d_rrr(qq, L2_3D, L3_3D), q_straight, 1e-6);
Jv_bent     = numerical_jacobian_fk3d(@(qq) fk_3d_rrr(qq, L2_3D, L3_3D), q_bent,     1e-6);

sv_straight = svd(Jv_straight);
sv_bent     = svd(Jv_bent);

% Manipulierbarkeit (Yoshikawa) für translatorische Jacobi: Produkt der
% Singularwerte. (Ohne Zusatzfunktion.)
w_straight = prod(sv_straight);
w_bent     = prod(sv_bent);

fprintf('Gerade Konfiguration: rank(Jv)=%d, min σ=%.3e, w=%.3e\n', rank(Jv_straight), min(sv_straight), w_straight);
fprintf('Gebeugt Konfiguration: rank(Jv)=%d, min σ=%.3e, w=%.3e\n', rank(Jv_bent),     min(sv_bent),     w_bent);

% Visualisiere Endeffektor-Transformationen in 3D
figure('Name', '3R: Basis→EE (gerade vs. gebeugt)', 'NumberTitle', 'off');
ax_3r = axes('Parent', gcf);
visualize_transformation(T_straight, ax_3r, '3R: gerade (nahe singular)');
hold(ax_3r, 'on');
visualize_transformation(T_bent, ax_3r, '3R: gebeugt');
hold(ax_3r, 'off');

%% 5. Inverse Kinematik nahe Singularitäten: Pseudoinverse vs. DLS
fprintf('\n=== Abschnitt 5: IK nahe Singularität – warum Dämpfung hilft ===\n');

% Wir demonstrieren das an der 2R-Planar-IK (Position-only):
%   p(q) = [x(q); y(q)]
%   Δq  = J^+ * e    (Pseudoinverse)
%   Δq  = J^T (J J^T + λ^2 I)^{-1} e   (DLS)
%
% In der Nähe einer Singularität wird J schlecht konditioniert.
% Dann kann J^+ extrem große Schritte liefern.

q0 = [deg2rad(20); deg2rad(1)];   % Start sehr nahe an q2=0° (fast gestreckt)
ptarget = [0.60; 0.05];          % Zielpunkt leicht außerhalb der "schlechten" Richtung

max_iter = 8;

% --- 5.1 Pseudoinverse Updates
q = q0;
fprintf('\nPseudoinverse-IK (ohne Dämpfung):\n');
for iter = 1:max_iter
    % Vorwärtskinematik (nur Position) für 2R-Planar
    q1c = q(1); q2c = q(2);
    pcur = [L1*cos(q1c) + L2*cos(q1c+q2c);
            L1*sin(q1c) + L2*sin(q1c+q2c)];
    J = jacobian_2r_planar(L1, L2, q1c, q2c);
    e = ptarget - pcur;
    dq = pinv(J) * e; % kann nahe Singularität sehr groß werden
    q = q + dq;
    fprintf('Iter %d: ||e||=%.3e, ||dq||=%.3e, q=[%.2f %.2f] deg\n', ...
        iter, norm(e), norm(dq), rad2deg(q(1)), rad2deg(q(2)));
end

% --- 5.2 DLS Updates
q = q0;
lambda = 0.10;
fprintf('\nDLS-IK (λ=%.2f):\n', lambda);
for iter = 1:max_iter
    q1c = q(1); q2c = q(2);
    pcur = [L1*cos(q1c) + L2*cos(q1c+q2c);
            L1*sin(q1c) + L2*sin(q1c+q2c)];
    J = jacobian_2r_planar(L1, L2, q1c, q2c);
    e = ptarget - pcur;
    % DLS-Update ohne zusätzliche Hilfsfunktion:
    % dq = J^T (J J^T + λ^2 I)^{-1} e
    dq = J.' * ((J*J.' + (lambda^2) * eye(2)) \ e);
    q = q + dq;
    fprintf('Iter %d: ||e||=%.3e, ||dq||=%.3e, q=[%.2f %.2f] deg\n', ...
        iter, norm(e), norm(dq), rad2deg(q(1)), rad2deg(q(2)));
end

%% 6. Anwendungsaufgaben (ohne Musterlösung)

%% Aufgabe 1 — Analytische Singularitätsbedingung 2R
% Leite det(J) für den 2R-Planararm her und zeige, dass
% det(J) = L1 * L2 * sin(q2).
% a) Schreibe einen kurzen MATLAB-Check, der det(J) numerisch für mehrere
%    q2-Werte berechnet.
% b) Plot det(J) und markiere die Nullstellen.

%% Aufgabe 2 — Konditionszahl und "Numerik-Warnsystem"
% Implementiere eine Funktion, die für eine gegebene Konfiguration q
% eine Warnung ausgibt, wenn cond(J) > 1e3.
% Teste das für q2 nahe 0° und q2 = 60°.

%% Aufgabe 3 — Geschwindigkeits-Ellipse automatisieren
% Schreibe eine Funktion `velocityEllipse2D(J, N)`, die N Punkte auf dem
% Einheitskreis im Gelenkraum abtastet und das resultierende Polygon in
% v-Raum zurückgibt.
% Plotte die Ellipse für mehrere q2-Werte und interpretiere die Form.

%% Aufgabe 4 — 3R-Kette: Singularitätssuche
% Verwende `jacobian_geometric` und untersuche die translatorische Jacobi Jv.
% a) Erzeuge zufällige Konfigurationen und speichere die kleinste Singularzahl.
% b) Finde eine Konfiguration mit sehr kleiner Singularzahl (nahe Singularität).
% c) Visualisiere diese Konfiguration (Basis→EE) mit `visualize_transformation`.

%% Aufgabe 5 — DLS-Parameterstudie
% Wiederhole Abschnitt 5 (DLS-IK) mit verschiedenen λ-Werten (z.B. 0.01, 0.05,
% 0.1, 0.3). Vergleiche
% - Konvergenzgeschwindigkeit
% - Schrittgröße ||dq||
% - Endfehler ||e||

%% Lokale Funktionsdefinitionen (aus Seminar 1 und Seminar 2)
% Hinweis: Es werden hier ausschließlich Funktionen eingebettet, die in
% `seminar_1.m` und `seminar_2.m` im Rahmen der Vorführungen entwickelt
% wurden.

function J = jacobian_2r_planar(L1, L2, q1, q2)
%JACOBIAN_2R_PLANAR Analytische 2x2-Jacobi für einen 2R-Planararm (XY).
%   v = J(q) * q_dot
%   v = [vx; vy]
%
% Wichtiger Lehrpunkt:
% - det(J) = L1*L2*sin(q2)
% - Rangverlust bei q2 = 0 oder π
J11 = -L1*sin(q1) - L2*sin(q1+q2);
J12 = -L2*sin(q1+q2);
J21 =  L1*cos(q1) + L2*cos(q1+q2);
J22 =  L2*cos(q1+q2);
J = [J11, J12; J21, J22];
end

function T = dh_transform(a, alpha, d, theta)
%DH_TRANSFORM Standard-DH-Transformationsmatrix (aus Seminar 2).
ca = cos(alpha); sa = sin(alpha);
ct = cos(theta); st = sin(theta);
T = [ ct, -st*ca,  st*sa, a*ct; ...
      st,  ct*ca, -ct*sa, a*st; ...
       0,     sa,     ca,    d; ...
       0,      0,      0,    1];
end

function T = fk_3d_rrr(q, L2_3D, L3_3D)
%FK_3D_RRR 3DOF-RRR Vorwärtskinematik (aus Seminar 2).
q1 = q(1); q2 = q(2); q3 = q(3);
T01 = dh_transform(0,  pi/2, 0, q1);
T12 = dh_transform(L2_3D, 0, 0, q2);
T23 = dh_transform(L3_3D, 0, 0, q3);
T = T01 * T12 * T23;
end

function J = numerical_jacobian_fk3d(fk_handle, q, delta)
%NUMERICAL_JACOBIAN_FK3D Numerische translatorische Jacobi (aus Seminar 2).
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

function T = rotX(theta)
%ROTX Rotation um X (aus Seminar 1).
T = eye(4);
T(2:3,2:3) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end

function T = rotY(theta)
%ROTY Rotation um Y (aus Seminar 1).
T = eye(4);
T([1 3], [1 3]) = [cos(theta), sin(theta); -sin(theta), cos(theta)];
end

function T = rotZ(theta)
%ROTZ Rotation um Z (aus Seminar 1).
T = eye(4);
T(1:2, 1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end

function T = transl(x, y, z)
%TRANSL Translation (aus Seminar 1).
T = eye(4);
T(1:3, 4) = [x; y; z];
end

function T = rpy2rotm(rpy)
%RPY2ROTM RPY -> Rotationsmatrix (aus Seminar 1).
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
T = rotZ(yaw) * rotY(pitch) * rotX(roll);
end

function rpy = rotm2rpy(T)
%ROTM2RPY Rotationsmatrix -> RPY (aus Seminar 1).
R = T(1:3, 1:3);
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
if abs(cos(pitch)) < 1e-8
    warning('Kardanische Blockade: Setze yaw für Lösung 0.');
    roll = atan2(R(1,2), R(2,2));
    yaw = 0;
else
    roll = atan2(R(3,2), R(3,3));
    yaw = atan2(R(2,1), R(1,1));
end
rpy = [roll, pitch, yaw];
end

function tprint(T, label)
%TPRINT Ausgabe einer Transformation (aus Seminar 1).
if nargin < 2, label = 'Transformation'; end
fprintf('\\n%s\\n', label);
fprintf('Position: [%.3f %.3f %.3f] m \\n', T(1:3, 4));
fprintf('Rotation: (3x3 oben-links):\\n');
disp(T(1:3, 1:3));
end

function visualize_transformation(T, varargin)
%VISUALIZE_TRANSFORMATION 3D-Visualisierung einer 4x4-Transformation (aus Seminar 1).

if ~isequal(size(T), [4, 4])
    error('visualize_transformation:InvalidSize', ...
        'Erwarte eine 4x4-Transformationsmatrix, erhalten wurde %s.', mat2str(size(T)));
end

ax = [];
plot_title = 'Visualisierung der Transformation';
for idx = 1:numel(varargin)
    candidate = varargin{idx};
    if isa(candidate, 'matlab.graphics.axis.Axes')
        ax = candidate;
    elseif (ischar(candidate) && ~isempty(candidate)) || (isstring(candidate) && isscalar(candidate))
        plot_title = char(candidate);
    else
        error('visualize_transformation:InvalidArgument', ...
            'Zusatzargument muss Achsenobjekt oder Titeltext sein.');
    end
end

if isempty(ax)
    fig = figure('Name', plot_title, 'NumberTitle', 'off'); %#ok<NASGU>
    ax = axes('Parent', fig);
end

hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 45, 25);
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
title(ax, plot_title);

scale = 0.15;
origin_base = [0; 0; 0];
origin_target = T(1:3, 4);
rotation = T(1:3, 1:3);

axis_colors = {'r', 'g', 'b'};
basis_vectors = eye(3);

for axis_idx = 1:3
    basis_dir = scale * basis_vectors(:, axis_idx);
    q = quiver3(ax, origin_base(1), origin_base(2), origin_base(3), ...
        basis_dir(1), basis_dir(2), basis_dir(3), 0, ...
        'Color', axis_colors{axis_idx}, 'LineWidth', 1.5, 'MaxHeadSize', 0.5); %#ok<NASGU>
    q.HandleVisibility = 'off';
end

for axis_idx = 1:3
    target_dir = scale * rotation(:, axis_idx);
    q = quiver3(ax, origin_target(1), origin_target(2), origin_target(3), ...
        target_dir(1), target_dir(2), target_dir(3), 0, ...
        'Color', axis_colors{axis_idx}, 'LineWidth', 2.2, 'MaxHeadSize', 0.6, ...
        'LineStyle', '--'); %#ok<NASGU>
    q.HandleVisibility = 'off';
end

l = plot3(ax, [origin_base(1), origin_target(1)], ...
           [origin_base(2), origin_target(2)], ...
           [origin_base(3), origin_target(3)], 'k:', 'LineWidth', 1.2); %#ok<NASGU>
l.HandleVisibility = 'off';

scatter3(ax, origin_base(1), origin_base(2), origin_base(3), 40, 'filled', ...
    'MarkerFaceColor', '#1f77b4', 'DisplayName', 'Basisursprung');
scatter3(ax, origin_target(1), origin_target(2), origin_target(3), 40, 'filled', ...
    'MarkerFaceColor', '#ff7f0e', 'DisplayName', 'Transformierter Ursprung');

if isempty(ax.Legend)
    legend(ax, 'show', 'Location', 'bestoutside');
end

hold(ax, 'off');
end
