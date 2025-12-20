% vis_sing_map.m
% ------------------------------------------------------------
% Visually rich demo: Singularity Map (3D) for a 6-DOF robot
%
% Ziel:
% - 6 Gelenke (R-R-R-R-R-R) mit kleiner UI (Slider)
% - Roboter wird in 3D angezeigt
% - Zusätzlich: "Singularitätskarte" im Arbeitsraum (3D-Punktwolke)
%   -> Punkte sind Endeffektorpositionen von (nahe-)singulären Konfigurationen
%   -> Für die aktuelle Konfiguration werden "nahe" Punkte hervorgehoben
%
% Einschränkung (wie von dir gefordert):
% - Nur MATLAB-Bordmittel
% - Nur Funktionen/Ideen, die in Seminar 1 & 2 vorkamen (DH/FK/Jacobi)
%   (keine Robotics Toolbox, keine externen Libraries)
%
% Nutzung:
% - In MATLAB ausführen:  vis_sing_map
%
% Hinweis zur Performance:
% - Die Punktwolke wird einmalig vorab berechnet (kann 5–30s dauern, je nach PC).
% - Während du Slider bewegst, wird nur die Hervorhebung aktualisiert.

clearvars;
clc;
close all;

fprintf("=== vis_sing_map: 6R Singularity Map Demo ===\n");

%% Robotermodell (Standard-DH)
% Wir wählen bewusst ein "PUMA-/Industriearm-artiges" 6R-Modell:
% - 3 Gelenke für Position
% - 3 Gelenke (Wrist) für Orientierung
% Das erzeugt einen komplexen 3D-Arbeitsraum und typische Singularitäten.
%
% DH-Konvention (Standard):
%   T_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
%
% Parameter (in Meter / Radiant):
a =     [ 0.00,  0.35,  0.25, 0.00, 0.00, 0.00];
alpha = [ pi/2, 0.00,  0.00,  pi/2, -pi/2, 0.00];
d =     [ 0.40, 0.00,  0.00, 0.30, 0.00, 0.10];

n0 = numel(a);
theta_offset = zeros(1, n0);

% Gelenkgrenzen (rad) – bewusst moderat gewählt (typisch: +-170° etc.)
q_min = deg2rad([-170, -120, -170, -190, -120, -360]);
q_max = deg2rad([ 170,  120,  170,  190,  120,  360]);

% Startkonfiguration
q0 = deg2rad([0, -30, 60, 0, 30, 0]);

% Consistency checks (helps when students change joint count)
if numel(alpha) ~= n0 || numel(d) ~= n0
    error('DH parameter vectors a/alpha/d must have the same length.');
end
if numel(q_min) ~= n0 || numel(q_max) ~= n0 || numel(q0) ~= n0
    error('q_min/q_max/q0 must match the number of joints.');
end

%% Precompute: Singularity Map (global)
% Kernidee:
% - Singularitäten sind Konfigurationen (q), nicht "Orte".
% - Für eine anschauliche Karte im Arbeitsraum projizieren wir jedoch
%   Konfigurationen -> Endeffektorposition p(q).
% - Wir sampeln viele q innerhalb der Grenzen und markieren die Punkte,
%   deren Jacobi-Matrix (6x6) nahe singulär ist.

rng(4); % reproduzierbar

N_samples = 25000;         % "stunning" aber noch handhabbar
sing_threshold = 1e-3;     % initiale Schwelle für min Singularwert σ_min(J)

fprintf("Precompute: %d Samples (das kann kurz dauern) ...\n", N_samples);

tStart = tic;

Q = zeros(N_samples, n0);
P = zeros(N_samples, 3);
SigmaMin = zeros(N_samples, 1);

for k = 1:N_samples
    q = q_min + rand(1,n0).*(q_max - q_min);
    Q(k,:) = q;

    [T_all, o_all, z_all] = fk_chain_dh(a, alpha, d, q + theta_offset);
    p = T_all{end}(1:3, 4).';
    P(k,:) = p;

    J = jacobian_geometric_from_fk(o_all, z_all); % 6x6
    s = svd(J);
    SigmaMin(k) = min(s);
end

elapsed = toc(tStart);
fprintf("Precompute fertig in %.1fs\n", elapsed);

% Wir behalten ALLE Samples (für UI-Filter später).
P_all = P;
Q_all = Q;
SigmaMin_all = SigmaMin;

score_all = -log10(max(SigmaMin_all, 1e-12));

% initiale Auswahl (nahe-)singulär
isSing = SigmaMin_all < sing_threshold;
fprintf("Singuläre Punkte (initial): %d / %d (%.1f%%)\n", nnz(isSing), N_samples, 100*nnz(isSing)/N_samples);

%% UI + Visualisierung
fig = figure('Name', 'vis_sing_map: 6R Singularity Map', 'NumberTitle', 'off', 'Color', 'w');

% Layout: große 3D-Achse links, UI rechts
ax = axes('Parent', fig, 'Position', [0.05 0.08 0.62 0.88]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 40, 20);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
title(ax, 'Singularity Map (Arbeitsraum) + aktueller Roboter');

% Punktwolke der Singularitäten
% - Wir nutzen viele Punkte, kleine Marker, Colormap "turbo" (modern, sehr klar)
if exist('turbo', 'file') == 2 || exist('turbo', 'builtin') == 5
    colormap(ax, turbo(256));
else
    colormap(ax, parula(256));
end

% Initialer Display-Subsample für bessere Interaktivität
display_fraction = 0.65; % 65% der Punkte anzeigen (später per UI einstellbar)
[P_show, score_show] = subsample_points(P_all(isSing,:), score_all(isSing), display_fraction);

sc_sing = scatter3(ax, P_show(:,1), P_show(:,2), P_show(:,3), 6, score_show, 'filled', ...
    'MarkerFaceAlpha', 0.20, 'MarkerEdgeAlpha', 0.00);
cb = colorbar(ax);
cb.Label.String = '-log10(\sigma_{min}(J)) (groß = "stärker" singulär)';

% Hervorhebung: nahe zur aktuellen Konfiguration (im Gelenkraum)
% -> Wir nehmen aus allen singulären Samples eine "Nähe" zu q_current
%    und modulieren die Größe/Alpha.
sc_focus = scatter3(ax, nan, nan, nan, 40, 'w', 'filled', ...
    'MarkerFaceAlpha', 0.90, 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);

% Roboter-Plot (per-Link farbig, wirkt deutlich "wertiger")
link_colors = lines(n0);
robot_links = gobjects(1,n0);
for i = 1:n0
    robot_links(i) = plot3(ax, nan, nan, nan, '-', 'LineWidth', 5, 'Color', link_colors(i,:));
end
robot_joints = scatter3(ax, nan, nan, nan, 70, [0.15 0.15 0.15], 'filled');
robot_ee = scatter3(ax, nan, nan, nan, 140, [0.85 0.1 0.1], 'filled');

% Endeffektor-Trail (optional)
% Hinweis: RGBA-Farben ([r g b a]) sind je nach MATLAB-Version nicht überall
% verfügbar. Wir nutzen deshalb eine normale RGB-Farbe.
ee_trail = plot3(ax, nan, nan, nan, '-', 'LineWidth', 1.5, 'Color', [0.85 0.1 0.1]);

% Geschwindigkeit-Ellipsoid (optional): v = Jv qdot, ||qdot||=1
% surf benötigt Matrizen für X/Y/Z. Wir initialisieren daher mit 2x2 NaNs.
ell = surf(ax, nan(2), nan(2), nan(2), 'FaceAlpha', 0.18, 'EdgeAlpha', 0.05, 'FaceColor', [0.2 0.2 0.2]);

% Educational overlay: Hauptachsen des Geschwindigkeit-Ellipsoids
ell_axes = gobjects(1,3);
ell_axes(1) = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2, 'Color', [0.85 0.25 0.25]);
ell_axes(2) = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2, 'Color', [0.25 0.75 0.25]);
ell_axes(3) = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2, 'Color', [0.25 0.45 0.85]);

% "Glam": Licht + Material (wirkt in MATLAB sehr gut)
camlight(ax, 'headlight');
lighting(ax, 'gouraud');

% UI Panels: getrennt für bessere Lesbarkeit (Students)
% Neues Ziel: Studierende sollen die Geometrie (aus URDF extrahiert) hier manuell eingeben.
panelGeom = uipanel('Parent', fig, 'Title', 'Robot Geometry (DH + Limits)', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.78 0.28 0.20]);
panelCtrl = uipanel('Parent', fig, 'Title', 'Controls', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.52 0.28 0.25]);
panelJoints = uipanel('Parent', fig, 'Title', 'Joint Angles (deg)', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.24 0.28 0.26]);
panelDiag = uipanel('Parent', fig, 'Title', 'Diagnostics', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.02 0.28 0.20]);

% Geometry table (students paste values they extracted from their URDF)
uicontrol('Parent', panelGeom, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.86 0.90 0.12], 'String', 'Enter DH (a, alpha, d, theta0) and joint limits (deg):', ...
    'HorizontalAlignment', 'left');

geomData = [a(:), rad2deg(alpha(:)), d(:), rad2deg(theta_offset(:)), rad2deg(q_min(:)), rad2deg(q_max(:))];

geom_table = uitable('Parent', panelGeom, 'Units', 'normalized', ...
    'Position', [0.05 0.28 0.90 0.56], ...
    'Data', geomData, ...
    'ColumnName', {'a [m]', 'alpha [deg]', 'd [m]', 'theta0 [deg]', 'qmin [deg]', 'qmax [deg]'}, ...
    'ColumnEditable', true(1,6), ...
    'RowName', arrayfun(@(i) sprintf('j%d', i), 1:n0, 'UniformOutput', false));

btn_add_joint = uicontrol('Parent', panelGeom, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.05 0.17 0.42 0.09], 'String', 'Add Joint');

btn_del_joint = uicontrol('Parent', panelGeom, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.53 0.17 0.42 0.09], 'String', 'Delete Last Joint');

btn_apply_geom = uicontrol('Parent', panelGeom, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.05 0.05 0.42 0.10], 'String', 'Apply Geometry');

btn_reset_geom = uicontrol('Parent', panelGeom, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.53 0.05 0.42 0.10], 'String', 'Reset Demo');

% Zusätzliche Controls (Threshold, Dichte, Toggles)
uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.92 0.90 0.06], 'String', 'Singularity Threshold (log10):', ...
    'HorizontalAlignment', 'left');

% Slider in log10-Skala: threshold = 10^(exp)
th_exp_init = log10(sing_threshold);
th_slider = uicontrol('Parent', panelCtrl, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.05 0.86 0.90 0.06], 'Min', -6, 'Max', -1, 'Value', th_exp_init);
th_label = uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.80 0.90 0.05], 'String', '', 'HorizontalAlignment', 'left');

uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.73 0.90 0.05], 'String', 'Display Density (% of points):', ...
    'HorizontalAlignment', 'left');
den_slider = uicontrol('Parent', panelCtrl, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.05 0.67 0.90 0.06], 'Min', 5, 'Max', 100, 'Value', 100*display_fraction);
den_label = uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.61 0.90 0.05], 'String', '', 'HorizontalAlignment', 'left');

cb_show_cloud = uicontrol('Parent', panelCtrl, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.54 0.90 0.06], 'Value', 1, 'String', 'Show singularity cloud');
cb_show_focus = uicontrol('Parent', panelCtrl, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.47 0.90 0.06], 'Value', 1, 'String', 'Highlight near-singular points');
cb_show_ell = uicontrol('Parent', panelCtrl, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.40 0.90 0.06], 'Value', 1, 'String', 'Show velocity ellipsoid');
cb_show_trail = uicontrol('Parent', panelCtrl, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.33 0.90 0.06], 'Value', 1, 'String', 'Show EE trail');

% Neuer UX-Control: Wie viele Punkte sollen als "nahe" hervorgehoben werden?
uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.25 0.90 0.06], 'String', 'Highlight Count (nearest in joint-space):', ...
    'HorizontalAlignment', 'left');
hl_slider = uicontrol('Parent', panelCtrl, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.05 0.19 0.90 0.06], 'Min', 0, 'Max', 500, 'Value', 120);
hl_label = uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.14 0.90 0.05], 'String', '', 'HorizontalAlignment', 'left');

% Presets: typische Situationen für Lehre
uicontrol('Parent', panelCtrl, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.08 0.20 0.05], 'String', 'Preset:', 'HorizontalAlignment', 'left');
preset_menu = uicontrol('Parent', panelCtrl, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.25 0.08 0.70 0.06], 'String', {'Home'}, ...
    'Value', 1);

btn_home = uicontrol('Parent', panelCtrl, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.05 0.00 0.43 0.07], 'String', 'Home');
btn_rand = uicontrol('Parent', panelCtrl, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.52 0.00 0.43 0.07], 'String', 'Random');

% Diagnostics plot: Singularwerte von J
diagAx = axes('Parent', panelDiag, 'Units', 'normalized', 'Position', [0.08 0.52 0.88 0.44]);
grid(diagAx, 'on');
title(diagAx, 'Singular values of J');
xlabel(diagAx, 'i');
ylabel(diagAx, '\sigma');
set(diagAx, 'YScale', 'log');
barSing = bar(diagAx, 1:n0, ones(1,n0));
barSing.FaceColor = [0.25 0.25 0.25];
barSing.EdgeColor = 'none';
diagAx.XLim = [0.5 max(1.5, n0+0.5)];
diagAx.YLim = [1e-6 1e2];

btn_export = uicontrol('Parent', panelDiag, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.62 0.94 0.34 0.06], 'String', 'Export...');

% Separate live figure: joint velocity education (linked to main UI)
[figVel, velAx, velBar, velTable, velText] = create_velocity_figure(fig, n0);

% Data in guidata
data = struct();
data.a = a; data.alpha = alpha; data.d = d; data.theta_offset = theta_offset;
data.q_min = q_min; data.q_max = q_max;
data.q0 = q0;
data.N_samples = N_samples;
data.ax = ax;
data.panelJoints = panelJoints;
data.sc_sing = sc_sing;
data.sc_focus = sc_focus;
data.robot_links = robot_links;
data.robot_joints = robot_joints;
data.robot_ee = robot_ee;
data.ee_trail = ee_trail;
data.ell = ell;
data.ell_axes = ell_axes;

% Geometry UI handles
data.panelGeom = panelGeom;
data.geom_table = geom_table;
data.btn_add_joint = btn_add_joint;
data.btn_del_joint = btn_del_joint;
data.btn_apply_geom = btn_apply_geom;
data.btn_reset_geom = btn_reset_geom;

data.P_all = P_all;
data.Q_all = Q_all;
data.SigmaMin_all = SigmaMin_all;
data.score_all = score_all;
data.sing_threshold = sing_threshold;
data.display_fraction = display_fraction;

% Stable subsampling (damit "Display Density" sichtbar wirkt und nicht flackert)
data.cloud_perm = [];

data.P_sing = P_all(isSing,:);
data.Q_sing = Q_all(isSing,:);
data.score_sing = score_all(isSing);

% Slider controls
sliders = gobjects(1,n0);
edits = gobjects(1,n0);
labels = gobjects(1,n0);

% Initial values
q_current = q0;

for i = 1:n0
    % Joint-Controls: eine Zeile pro Gelenk (in eigenem Panel)
    y = 0.83 - (i-1)*(0.155 * 6 / max(6, n0));

    labels(i) = uicontrol('Parent', panelJoints, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05 y 0.20 0.12], 'String', sprintf('q%d [deg]', i), 'HorizontalAlignment', 'left');

    sliders(i) = uicontrol('Parent', panelJoints, 'Style', 'slider', 'Units', 'normalized', ...
        'Position', [0.26 y 0.52 0.12], 'Min', q_min(i), 'Max', q_max(i), 'Value', q_current(i));

    edits(i) = uicontrol('Parent', panelJoints, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [0.80 y 0.17 0.12], 'String', sprintf('%.1f', rad2deg(q_current(i))));

    % Callback: slider -> edit + update
    sliders(i).Callback = @(src,~) onSliderChanged(src, i);

    % Callback: edit -> slider + update
    edits(i).Callback = @(src,~) onEditChanged(src, i);
end

% Info text
infoText = uicontrol('Parent', panelDiag, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.08 0.04 0.88 0.44], 'String', 'Ziehe Slider oder tippe Grad-Wert.', ...
    'HorizontalAlignment', 'left', 'FontSize', 9);

data.sliders = sliders;
data.edits = edits;
data.labels = labels;
data.infoText = infoText;

data.th_slider = th_slider;
data.th_label = th_label;
data.den_slider = den_slider;
data.den_label = den_label;
data.hl_slider = hl_slider;
data.hl_label = hl_label;
data.highlightK = round(hl_slider.Value);
data.preset_menu = preset_menu;
data.presets = build_presets(data.q_min, data.q_max, data.q0);
data.diagAx = diagAx;
data.barSing = barSing;
data.btn_export = btn_export;
data.figVel = figVel;
data.velAx = velAx;
data.velBar = velBar;
data.velTable = velTable;
data.velText = velText;
data.cb_show_cloud = cb_show_cloud;
data.cb_show_focus = cb_show_focus;
data.cb_show_ell = cb_show_ell;
data.cb_show_trail = cb_show_trail;
data.btn_home = btn_home;
data.btn_rand = btn_rand;

% Trail state
data.trail_maxlen = 250;
data.trail_pts = zeros(0,3);

guidata(fig, data);

% Initialize preset menu entries
data0 = guidata(fig);
set(data0.preset_menu, 'String', {data0.presets.name}, 'Value', 1);
guidata(fig, data0);

% First draw
updateScene(fig);

% UI callbacks (nach guidata, damit updateScene Zugriff hat)
th_slider.Callback = @(src,~) onThresholdChanged(src);
den_slider.Callback = @(src,~) onDensityChanged(src);
hl_slider.Callback = @(src,~) onHighlightChanged(src);
preset_menu.Callback = @(src,~) onPresetChanged(src);
cb_show_cloud.Callback = @(src,~) onToggleChanged(src);
cb_show_focus.Callback = @(src,~) onToggleChanged(src);
cb_show_ell.Callback = @(src,~) onToggleChanged(src);
cb_show_trail.Callback = @(src,~) onToggleChanged(src);

btn_home.Callback = @(src,~) onHome(src);
btn_rand.Callback = @(src,~) onRandom(src);

btn_apply_geom.Callback = @(src,~) onApplyGeometry(src);
btn_reset_geom.Callback = @(src,~) onResetGeometry(src);
btn_add_joint.Callback = @(src,~) onAddJoint(src);
btn_del_joint.Callback = @(src,~) onDeleteJoint(src);

btn_export.Callback = @(src,~) onExport(src);

%% Callbacks
function onSliderChanged(src, jointIdx)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    q = src.Value;
    dataLocal.edits(jointIdx).String = sprintf('%.1f', rad2deg(q));
    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onEditChanged(src, jointIdx)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);

    % Erwartet eine Zahl in Grad
    valDeg = str2double(src.String);
    if isnan(valDeg)
        src.String = sprintf('%.1f', rad2deg(dataLocal.sliders(jointIdx).Value));
        return;
    end

    q = deg2rad(valDeg);
    q = min(max(q, dataLocal.q_min(jointIdx)), dataLocal.q_max(jointIdx));
    dataLocal.sliders(jointIdx).Value = q;
    src.String = sprintf('%.1f', rad2deg(q));

    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onThresholdChanged(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    expVal = src.Value;
    dataLocal.sing_threshold = 10.^expVal;
    % Threshold ändert die Menge singulärer Punkte -> neue Permutation
    dataLocal.cloud_perm = [];
    guidata(figLocal, dataLocal);
    updateCloud(figLocal);
    updateScene(figLocal);
end

function onDensityChanged(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    dataLocal.display_fraction = max(min(src.Value/100, 1), 0.01);
    guidata(figLocal, dataLocal);
    updateCloud(figLocal);
    updateScene(figLocal);
end

function onHighlightChanged(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    dataLocal.highlightK = round(src.Value);
    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onToggleChanged(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    % Sichtbarkeit (wichtig: Fokus ist NICHT die Wolke)
    dataLocal.sc_sing.Visible = onoff(dataLocal.cb_show_cloud.Value);
    dataLocal.sc_focus.Visible = onoff(dataLocal.cb_show_focus.Value);
    dataLocal.ell.Visible = onoff(dataLocal.cb_show_ell.Value);
    if isfield(dataLocal, 'ell_axes')
        for kk = 1:numel(dataLocal.ell_axes)
            if isgraphics(dataLocal.ell_axes(kk))
                dataLocal.ell_axes(kk).Visible = onoff(dataLocal.cb_show_ell.Value);
            end
        end
    end
    dataLocal.ee_trail.Visible = onoff(dataLocal.cb_show_trail.Value);
    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onHome(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    qhome = dataLocal.q0;
    for ii = 1:numel(dataLocal.sliders)
        dataLocal.sliders(ii).Value = qhome(ii);
        dataLocal.edits(ii).String = sprintf('%.1f', rad2deg(qhome(ii)));
    end
    dataLocal.trail_pts = zeros(0,3);
    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onRandom(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    n = numel(dataLocal.q_min);
    q = dataLocal.q_min + rand(1,n).*(dataLocal.q_max - dataLocal.q_min);
    for ii = 1:numel(dataLocal.sliders)
        dataLocal.sliders(ii).Value = q(ii);
        dataLocal.edits(ii).String = sprintf('%.1f', rad2deg(q(ii)));
    end
    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onPresetChanged(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);
    v = src.Value;

    if isfield(dataLocal, 'presets') && ~isempty(dataLocal.presets) && v >= 1 && v <= numel(dataLocal.presets)
        q = dataLocal.presets(v).q;
    else
        q = dataLocal.q0;
    end

    for ii = 1:numel(dataLocal.sliders)
        q(ii) = min(max(q(ii), dataLocal.q_min(ii)), dataLocal.q_max(ii));
        dataLocal.sliders(ii).Value = q(ii);
        dataLocal.edits(ii).String = sprintf('%.1f', rad2deg(q(ii)));
    end

    guidata(figLocal, dataLocal);
    updateScene(figLocal);
end

function onApplyGeometry(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);

    tbl = dataLocal.geom_table.Data;
    if isempty(tbl) || size(tbl,2) ~= 6 || size(tbl,1) < 1
        errordlg('Geometry table must have 6 columns (a, alpha, d, theta0, qmin, qmax) and at least 1 row.', 'Invalid geometry');
        return;
    end

    % Some MATLAB versions return table data as cell arrays after edits
    if iscell(tbl)
        try
            tbl = cell2mat(tbl);
        catch
            errordlg('Geometry table contains non-numeric values.', 'Invalid geometry');
            return;
        end
    end

    if any(~isfinite(tbl(:)))
        errordlg('Geometry table contains non-numeric values.', 'Invalid geometry');
        return;
    end

    nNew = size(tbl,1);

    a_new = tbl(:,1).';
    alpha_new = deg2rad(tbl(:,2).');
    d_new = tbl(:,3).';
    theta0_new = deg2rad(tbl(:,4).');
    qmin_new = deg2rad(tbl(:,5).');
    qmax_new = deg2rad(tbl(:,6).');

    if any(qmax_new <= qmin_new)
        errordlg('Each joint must satisfy qmax > qmin.', 'Invalid limits');
        return;
    end

    dataLocal.a = a_new;
    dataLocal.alpha = alpha_new;
    dataLocal.d = d_new;
    dataLocal.theta_offset = theta0_new;
    dataLocal.q_min = qmin_new;
    dataLocal.q_max = qmax_new;

    % Keep row names consistent
    dataLocal.geom_table.RowName = arrayfun(@(i) sprintf('j%d', i), 1:nNew, 'UniformOutput', false);

    % Update "home" as mid-range (safe default) unless existing home is still valid
    q0_new = 0.5*(qmin_new + qmax_new);
    if isfield(dataLocal, 'q0') && numel(dataLocal.q0) == nNew
        q0_try = min(max(dataLocal.q0, qmin_new), qmax_new);
        q0_new = q0_try;
    end
    dataLocal.q0 = q0_new;

    % If joint count changed, rebuild slider UI and robot link objects
    if ~isfield(dataLocal, 'sliders') || numel(dataLocal.sliders) ~= nNew
        guidata(figLocal, dataLocal);
        rebuildJointAndRobotUI(figLocal, nNew);
        dataLocal = guidata(figLocal);
    end

    % Update slider ranges + clamp values
    for ii = 1:numel(dataLocal.sliders)
        dataLocal.sliders(ii).Min = dataLocal.q_min(ii);
        dataLocal.sliders(ii).Max = dataLocal.q_max(ii);
        qv = min(max(dataLocal.sliders(ii).Value, dataLocal.q_min(ii)), dataLocal.q_max(ii));
        dataLocal.sliders(ii).Value = qv;
        dataLocal.edits(ii).String = sprintf('%.1f', rad2deg(qv));
    end

    % Rebuild presets and menu
    dataLocal.presets = build_presets(dataLocal.q_min, dataLocal.q_max, dataLocal.q0);
    set(dataLocal.preset_menu, 'String', {dataLocal.presets.name}, 'Value', 1);

    % Recompute singularity cloud for new geometry
    guidata(figLocal, dataLocal);
    recomputeCloud(figLocal);
end

function onAddJoint(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);

    tbl = dataLocal.geom_table.Data;
    if isempty(tbl)
        tbl = zeros(0,6);
    end
    if iscell(tbl)
        try
            tbl = cell2mat(tbl);
        catch
            tbl = zeros(0,6);
        end
    end
    if size(tbl,2) ~= 6
        errordlg('Geometry table must have 6 columns.', 'Add joint');
        return;
    end

    % Default new joint row (students will edit)
    newRow = [0, 0, 0, 0, -180, 180];
    tbl = [tbl; newRow];

    dataLocal.geom_table.Data = tbl;
    dataLocal.geom_table.RowName = arrayfun(@(i) sprintf('j%d', i), 1:size(tbl,1), 'UniformOutput', false);
    guidata(figLocal, dataLocal);
end

function onDeleteJoint(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);

    tbl = dataLocal.geom_table.Data;
    if iscell(tbl)
        try
            tbl = cell2mat(tbl);
        catch
            errordlg('Geometry table contains non-numeric values.', 'Delete joint');
            return;
        end
    end

    if isempty(tbl) || size(tbl,1) <= 1
        errordlg('Robot must have at least 1 joint.', 'Delete joint');
        return;
    end

    tbl = tbl(1:end-1, :);
    dataLocal.geom_table.Data = tbl;
    dataLocal.geom_table.RowName = arrayfun(@(i) sprintf('j%d', i), 1:size(tbl,1), 'UniformOutput', false);
    guidata(figLocal, dataLocal);
end

function rebuildJointAndRobotUI(figLocal, nNew)
    dataLocal = guidata(figLocal);

    % --- Robot links ---
    if isfield(dataLocal, 'robot_links')
        for ii = 1:numel(dataLocal.robot_links)
            if isgraphics(dataLocal.robot_links(ii))
                delete(dataLocal.robot_links(ii));
            end
        end
    end
    link_colors = lines(nNew);
    dataLocal.robot_links = gobjects(1, nNew);
    for i = 1:nNew
        dataLocal.robot_links(i) = plot3(dataLocal.ax, nan, nan, nan, '-', 'LineWidth', 5, 'Color', link_colors(i,:));
    end

    % --- Joint controls ---
    if isfield(dataLocal, 'sliders')
        for ii = 1:numel(dataLocal.sliders)
            if isgraphics(dataLocal.sliders(ii))
                delete(dataLocal.sliders(ii));
            end
        end
    end
    if isfield(dataLocal, 'edits')
        for ii = 1:numel(dataLocal.edits)
            if isgraphics(dataLocal.edits(ii))
                delete(dataLocal.edits(ii));
            end
        end
    end
    if isfield(dataLocal, 'labels')
        for ii = 1:numel(dataLocal.labels)
            if isgraphics(dataLocal.labels(ii))
                delete(dataLocal.labels(ii));
            end
        end
    end

    dataLocal.sliders = gobjects(1, nNew);
    dataLocal.edits = gobjects(1, nNew);
    dataLocal.labels = gobjects(1, nNew);

    % Ensure q0 matches n
    if ~isfield(dataLocal, 'q0') || numel(dataLocal.q0) ~= nNew
        dataLocal.q0 = 0.5*(dataLocal.q_min + dataLocal.q_max);
    end

    for i = 1:nNew
        y = 0.83 - (i-1)*(0.155 * 6 / max(6, nNew));
        dataLocal.labels(i) = uicontrol('Parent', dataLocal.panelJoints, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.05 y 0.20 0.12], 'String', sprintf('q%d [deg]', i), 'HorizontalAlignment', 'left');

        dataLocal.sliders(i) = uicontrol('Parent', dataLocal.panelJoints, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.26 y 0.52 0.12], 'Min', dataLocal.q_min(i), 'Max', dataLocal.q_max(i), 'Value', dataLocal.q0(i));

        dataLocal.edits(i) = uicontrol('Parent', dataLocal.panelJoints, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [0.80 y 0.17 0.12], 'String', sprintf('%.1f', rad2deg(dataLocal.q0(i))));

        dataLocal.sliders(i).Callback = @(src,~) onSliderChanged(src, i);
        dataLocal.edits(i).Callback = @(src,~) onEditChanged(src, i);
    end

    % If the diagnostics bar has the wrong length, recreate it lazily in updateScene.
    % Keep the separate velocity figure in sync with joint count.
    [dataLocal.figVel, dataLocal.velAx, dataLocal.velBar, dataLocal.velTable, dataLocal.velText] = create_velocity_figure(figLocal, nNew);
    guidata(figLocal, dataLocal);
end

function onExport(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);

    [file, path] = uiputfile('*.png', 'Export main figure as...', 'vis_sing_map.png');
    if isequal(file, 0)
        return;
    end

    basePng = fullfile(path, file);
    [~, baseName, ~] = fileparts(basePng);

    % Compute current kinematics
    q = get_current_q(dataLocal);
    [T_all, o_all, z_all] = fk_chain_dh(dataLocal.a, dataLocal.alpha, dataLocal.d, q + dataLocal.theta_offset);
    ee = T_all{end}(1:3,4).';
    J = jacobian_geometric_from_fk(o_all, z_all);
    s = svd(J);
    Jv = J(1:3, :);
    infl = vecnorm(Jv, 2, 1);

    % Export main figure
    exportedMain = false;
    % 1) Best effort: export UI-containing figure via exportapp (newer MATLAB)
    if exist('exportapp', 'file') == 2
        try
            exportapp(figLocal, basePng);
            exportedMain = true;
        catch
        end
    end
    % 2) Next best: export the 3D axes only (robust, excludes UI controls)
    if ~exportedMain && exist('exportgraphics', 'file') == 2
        try
            exportgraphics(dataLocal.ax, basePng, 'Resolution', 300);
            exportedMain = true;
        catch
        end
    end
    % 3) Last resort: rasterize via getframe
    if ~exportedMain
        try
            fr = getframe(figLocal);
            imwrite(fr.cdata, basePng);
            exportedMain = true;
        catch
        end
    end
    if ~exportedMain
        errordlg('Export failed on this MATLAB version.', 'Export');
        return;
    end

    % Export joint velocity influence figure
    try
        f2 = figure('Visible', 'off', 'Color', 'w', 'Name', 'Joint velocity influence');
        ax2 = axes('Parent', f2);
        grid(ax2, 'on');
        bar(ax2, 1:numel(infl), infl, 'FaceColor', [0.20 0.45 0.85], 'EdgeColor', 'none');
        title(ax2, '||J_v(:,i)|| for unit joint rate');
        xlabel(ax2, 'joint i');
        ylabel(ax2, 'm/s per rad/s');
        out2 = fullfile(path, [baseName '_joint_velocity.png']);
        if exist('exportgraphics', 'file') == 2
            exportgraphics(f2, out2, 'Resolution', 300);
        else
            saveas(f2, out2);
        end
        close(f2);
    catch
        % ignore
    end

    % Export singular values figure
    try
        f3 = figure('Visible', 'off', 'Color', 'w', 'Name', 'Singular values');
        ax3 = axes('Parent', f3);
        grid(ax3, 'on');
        set(ax3, 'YScale', 'log');
        ss = max(s(:).', 1e-12);
        bar(ax3, 1:numel(ss), ss, 'FaceColor', [0.25 0.25 0.25], 'EdgeColor', 'none');
        title(ax3, 'Singular values of J');
        xlabel(ax3, 'i');
        ylabel(ax3, '\sigma');
        out3 = fullfile(path, [baseName '_singular_values.png']);
        if exist('exportgraphics', 'file') == 2
            exportgraphics(f3, out3, 'Resolution', 300);
        else
            saveas(f3, out3);
        end
        close(f3);
    catch
        % ignore
    end

    % Export MAT data bundle
    try
        exportData = struct();
        exportData.timestamp = datestr(now);
        exportData.a = dataLocal.a;
        exportData.alpha = dataLocal.alpha;
        exportData.d = dataLocal.d;
        exportData.theta_offset = dataLocal.theta_offset;
        exportData.q_min = dataLocal.q_min;
        exportData.q_max = dataLocal.q_max;
        exportData.q = q;
        exportData.ee = ee;
        exportData.J = J;
        exportData.singular_values = s;
        exportData.sigma_min = min(s);
        exportData.condJ = cond(J);
        exportData.influence_norms = infl;
        exportData.sing_threshold = dataLocal.sing_threshold;
        exportData.cloud_points = size(dataLocal.P_sing, 1);

        save(fullfile(path, [baseName '_data.mat']), 'exportData');
    catch
        % ignore
    end
end

function q = get_current_q(dataLocal)
    n = numel(dataLocal.sliders);
    q = zeros(1, n);
    for ii = 1:n
        q(ii) = dataLocal.sliders(ii).Value;
    end
end

function [figVel, velAx, velBar, velTable, velText] = create_velocity_figure(figMain, n)
%CREATE_VELOCITY_FIGURE Create or reuse a live-linked joint velocity figure.

dataMain = guidata(figMain);
figVel = [];
velAx = [];
velBar = [];
velTable = [];
velText = [];

if isfield(dataMain, 'figVel') && ~isempty(dataMain.figVel) && isgraphics(dataMain.figVel)
    figVel = dataMain.figVel;
end

% Create if needed
if isempty(figVel)
    figVel = figure('Name', 'Joint Velocity (linked)', 'NumberTitle', 'off', 'Color', 'w');
end

% Preserve existing qdot values if possible
qdotOld = [];
if isfield(dataMain, 'velTable') && ~isempty(dataMain.velTable) && isgraphics(dataMain.velTable)
    try
        td = dataMain.velTable.Data;
        if iscell(td)
            td = cell2mat(td);
        end
        if size(td,2) == 1
            qdotOld = td(:);
        end
    catch
    end
end

% Clear and rebuild contents (simple and robust)
clf(figVel);

velAx = axes('Parent', figVel, 'Units', 'normalized', 'Position', [0.10 0.40 0.85 0.55]);
grid(velAx, 'on');
title(velAx, '||J_v(:,i)|| for unit joint rate');
xlabel(velAx, 'joint i');
ylabel(velAx, 'm/s per rad/s');
velBar = bar(velAx, 1:n, zeros(1,n), 'FaceColor', [0.20 0.45 0.85], 'EdgeColor', 'none');
velAx.XLim = [0.5 max(1.5, n+0.5)];

qdot = zeros(n,1);
if ~isempty(qdotOld)
    qdot(1:min(n, numel(qdotOld))) = qdotOld(1:min(n, numel(qdotOld)));
end

velTable = uitable('Parent', figVel, 'Units', 'normalized', 'Position', [0.10 0.05 0.55 0.28], ...
    'Data', qdot, ...
    'ColumnName', {'qdot [rad/s]'}, ...
    'ColumnEditable', true, ...
    'RowName', arrayfun(@(i) sprintf('qdot%d', i), 1:n, 'UniformOutput', false));
velTable.UserData = figMain;
velTable.CellEditCallback = @(src,~) onVelQdotEdited(src);

velText = uicontrol('Parent', figVel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.67 0.05 0.28 0.28], ...
    'String', 'v = [0 0 0] m/s\n|v| = 0\nomega = [0 0 0] rad/s\n|omega| = 0', ...
    'HorizontalAlignment', 'left');

% Store back to main guidata so updateScene can drive it
dataMain.figVel = figVel;
dataMain.velAx = velAx;
dataMain.velBar = velBar;
dataMain.velTable = velTable;
dataMain.velText = velText;
guidata(figMain, dataMain);
end

function onVelQdotEdited(src)
% Update main scene when qdot table changes
figMain = [];
try
    figMain = src.UserData;
catch
end
if ~isempty(figMain) && isgraphics(figMain)
    updateScene(figMain);
end
end

function onResetGeometry(src)
    figLocal = ancestor(src, 'figure');
    dataLocal = guidata(figLocal);

    % restore demo defaults (same as top of script)
    a_new =     [ 0.00,  0.35,  0.25, 0.00, 0.00, 0.00];
    alpha_new = [ pi/2, 0.00,  0.00,  pi/2, -pi/2, 0.00];
    d_new =     [ 0.40, 0.00,  0.00, 0.30, 0.00, 0.10];
    theta0_new = zeros(1,6);
    qmin_new = deg2rad([-170, -120, -170, -190, -120, -360]);
    qmax_new = deg2rad([ 170,  120,  170,  190,  120,  360]);
    q0_new = deg2rad([0, -30, 60, 0, 30, 0]);

    dataLocal.a = a_new;
    dataLocal.alpha = alpha_new;
    dataLocal.d = d_new;
    dataLocal.theta_offset = theta0_new;
    dataLocal.q_min = qmin_new;
    dataLocal.q_max = qmax_new;
    dataLocal.q0 = q0_new;

    % Update table
    dataLocal.geom_table.Data = [a_new(:), rad2deg(alpha_new(:)), d_new(:), rad2deg(theta0_new(:)), rad2deg(qmin_new(:)), rad2deg(qmax_new(:))];
    dataLocal.geom_table.RowName = arrayfun(@(i) sprintf('j%d', i), 1:numel(a_new), 'UniformOutput', false);

    % Rebuild slider UI and robot link objects for the default joint count
    guidata(figLocal, dataLocal);
    rebuildJointAndRobotUI(figLocal, numel(a_new));
    dataLocal = guidata(figLocal);

    % Update slider ranges + set to home
    for ii = 1:numel(dataLocal.sliders)
        dataLocal.sliders(ii).Min = dataLocal.q_min(ii);
        dataLocal.sliders(ii).Max = dataLocal.q_max(ii);
        dataLocal.sliders(ii).Value = dataLocal.q0(ii);
        dataLocal.edits(ii).String = sprintf('%.1f', rad2deg(dataLocal.q0(ii)));
    end

    dataLocal.presets = build_presets(dataLocal.q_min, dataLocal.q_max, dataLocal.q0);
    set(dataLocal.preset_menu, 'String', {dataLocal.presets.name}, 'Value', 1);

    guidata(figLocal, dataLocal);
    recomputeCloud(figLocal);
end

function updateCloud(figLocal)
    dataLocal = guidata(figLocal);
    th = dataLocal.sing_threshold;
    frac = dataLocal.display_fraction;

    isSing = dataLocal.SigmaMin_all < th;
    dataLocal.P_sing = dataLocal.P_all(isSing,:);
    dataLocal.Q_sing = dataLocal.Q_all(isSing,:);
    dataLocal.score_sing = dataLocal.score_all(isSing);

    % Stable subsampling: gleiche Reihenfolge beibehalten, wenn nur die Dichte
    % geändert wird (sonst wirkt der Slider "tot" / flackernd).
    n = size(dataLocal.P_sing, 1);
    if isempty(dataLocal.cloud_perm) || numel(dataLocal.cloud_perm) ~= n
        if n > 0
            dataLocal.cloud_perm = randperm(n);
        else
            dataLocal.cloud_perm = [];
        end
    end
    k = 0;
    if n > 0
        k = max(1, round(frac * n));
        idx = dataLocal.cloud_perm(1:k);
        P_show = dataLocal.P_sing(idx, :);
        score_show = dataLocal.score_sing(idx, :);
    else
        P_show = zeros(0,3);
        score_show = zeros(0,1);
    end

    dataLocal.sc_sing.XData = P_show(:,1);
    dataLocal.sc_sing.YData = P_show(:,2);
    dataLocal.sc_sing.ZData = P_show(:,3);
    dataLocal.sc_sing.CData = score_show;

    % Labels aktualisieren
    dataLocal.th_label.String = sprintf('threshold = 10^{%.2f} = %.2e  |  points: %d', log10(th), th, size(dataLocal.P_sing,1));
    dataLocal.den_label.String = sprintf('display = %.0f%%  |  shown: %d', 100*frac, size(P_show,1));
    if ~isfield(dataLocal, 'highlightK') || isempty(dataLocal.highlightK)
        dataLocal.highlightK = round(dataLocal.hl_slider.Value);
    end
    dataLocal.hl_label.String = sprintf('highlight K = %d', dataLocal.highlightK);

    guidata(figLocal, dataLocal);
end

function recomputeCloud(figLocal)
    dataLocal = guidata(figLocal);

    wb = waitbar(0, 'Recomputing singularity cloud (this may take a moment)...');
    cleanup = onCleanup(@() safe_close_waitbar(wb));

    N = dataLocal.N_samples;
    n = numel(dataLocal.q_min);
    Q = zeros(N, n);
    P = zeros(N, 3);
    SigmaMin = zeros(N, 1);

    tStart = tic;
    for k = 1:N
        q = dataLocal.q_min + rand(1,n).*(dataLocal.q_max - dataLocal.q_min);
        Q(k,:) = q;

        [T_all, o_all, z_all] = fk_chain_dh(dataLocal.a, dataLocal.alpha, dataLocal.d, q + dataLocal.theta_offset);
        P(k,:) = T_all{end}(1:3, 4).';

        J = jacobian_geometric_from_fk(o_all, z_all);
        s = svd(J);
        SigmaMin(k) = min(s);

        if mod(k, 250) == 0
            waitbar(k/N, wb);
        end
    end

    dataLocal.P_all = P;
    dataLocal.Q_all = Q;
    dataLocal.SigmaMin_all = SigmaMin;
    dataLocal.score_all = -log10(max(SigmaMin, 1e-12));

    elapsed = toc(tStart);
    fprintf('Recompute fertig in %.1fs\n', elapsed);

    % Reset subsampling cache + update filtered view
    dataLocal.cloud_perm = [];
    guidata(figLocal, dataLocal);
    updateCloud(figLocal);
    updateScene(figLocal);
end

function safe_close_waitbar(wb)
    if ~isempty(wb) && isgraphics(wb)
        close(wb);
    end
end

function presets = build_presets(q_min, q_max, q0)
%BUILD_PRESETS Create a richer preset list (safe + teaching-focused).
presets = struct('name', {}, 'q', {});

qmin = q_min(:).';
qmax = q_max(:).';
nj = numel(qmin);

% Helpers
clip = @(q) min(max(q, qmin), qmax);
qmid = 0.5*(qmin + qmax);
qlo = qmin + 0.05*(qmax - qmin);
qhi = qmax - 0.05*(qmax - qmin);

presets(end+1) = struct('name', 'Home', 'q', clip(q0));
presets(end+1) = struct('name', 'All zeros', 'q', clip(zeros(1,nj)));
presets(end+1) = struct('name', 'Mid-range', 'q', clip(qmid));

% Classic teaching poses (still useful even if geometry differs)
if nj >= 5
    q_wrist = clip(q0);
    q_wrist(5) = 0;
    presets(end+1) = struct('name', 'Wrist singular (q5=0)', 'q', q_wrist);
end

if nj >= 3
    q_elbow = clip(q0);
    q_elbow(2) = 0;
    q_elbow(3) = 0;
    presets(end+1) = struct('name', 'Elbow stretched (q2=0,q3=0)', 'q', q_elbow);

    q_fold = clip(q0);
    q_fold(2) = -pi/2;
    q_fold(3) = pi/2;
    presets(end+1) = struct('name', 'Folded elbow (q2=-90,q3=90)', 'q', q_fold);
end

if nj >= 1
    q_sh = clip(q0);
    q_sh(1) = pi/2;
    presets(end+1) = struct('name', 'Shoulder twist (q1=90)', 'q', q_sh);
end

% Near limits (students often want to see what happens)
presets(end+1) = struct('name', 'Near lower limits', 'q', clip(qlo));
presets(end+1) = struct('name', 'Near upper limits', 'q', clip(qhi));

% Alternating pose
alt = qmid;
for i = 1:nj
    r = 0.35*(qmax(i) - qmin(i));
    if mod(i,2)==0
        alt(i) = qmid(i) + r;
    else
        alt(i) = qmid(i) - r;
    end
end
presets(end+1) = struct('name', 'Alternating pose', 'q', clip(alt));
end

%% Scene update
function updateScene(figLocal)
    dataLocal = guidata(figLocal);

    % Ensure cloud labels are up-to-date
    if isempty(dataLocal.th_label.String) || isempty(dataLocal.den_label.String)
        updateCloud(figLocal);
        dataLocal = guidata(figLocal);
    end

    % Read q from sliders
    n = numel(dataLocal.sliders);
    q = zeros(1,n);
    for ii = 1:n
        q(ii) = dataLocal.sliders(ii).Value;
    end

    % FK chain for visualization
    [T_all, o_all, ~] = fk_chain_dh(dataLocal.a, dataLocal.alpha, dataLocal.d, q + dataLocal.theta_offset);

    % Joint positions
    pts = zeros(n+1,3);
    for j = 1:(n+1)
        pts(j,:) = o_all{j}.';
    end

    % Update robot plot
    for i = 1:n
        dataLocal.robot_links(i).XData = [pts(i,1) pts(i+1,1)];
        dataLocal.robot_links(i).YData = [pts(i,2) pts(i+1,2)];
        dataLocal.robot_links(i).ZData = [pts(i,3) pts(i+1,3)];
    end

    dataLocal.robot_joints.XData = pts(:,1);
    dataLocal.robot_joints.YData = pts(:,2);
    dataLocal.robot_joints.ZData = pts(:,3);

    ee = T_all{end}(1:3,4).';
    dataLocal.robot_ee.XData = ee(1);
    dataLocal.robot_ee.YData = ee(2);
    dataLocal.robot_ee.ZData = ee(3);

    % "Relevanz" im Gelenkraum: welche singulären Samples sind nahe am aktuellen q?
    % Wichtig: Nur die NÄCHSTEN K hervorheben (sonst wirkt es wie "alles").
    if dataLocal.cb_show_focus.Value ~= 1 || isempty(dataLocal.Q_sing)
        dist = [];
        idx = [];
        Pf = zeros(0,3);
        K = 0;
    else
        dq = wrapToPi_local(dataLocal.Q_sing - q);
        dist = sqrt(sum(dq.^2, 2));

        if ~isfield(dataLocal, 'highlightK') || isempty(dataLocal.highlightK)
            dataLocal.highlightK = round(dataLocal.hl_slider.Value);
        end

        K = min(max(0, dataLocal.highlightK), size(dataLocal.Q_sing, 1));
        if K == 0
            idx = [];
            Pf = zeros(0,3);
        else
            [~, idx] = mink(dist, K);
            Pf = dataLocal.P_sing(idx, :);
        end
    end

    % Größe abhängig von Distanz
    if isempty(Pf)
        sz = [];
    else
        dsel = dist(idx);
        d0 = max(prctile(dsel, 60), 1e-6);
        w = exp(-(dsel./d0).^2);
        sz = 30 + 140*w;
    end

    dataLocal.sc_focus.XData = Pf(:,1);
    dataLocal.sc_focus.YData = Pf(:,2);
    dataLocal.sc_focus.ZData = Pf(:,3);
    if isempty(Pf)
        dataLocal.sc_focus.SizeData = 40;
    else
        dataLocal.sc_focus.SizeData = sz;
    end

    % EE trail (optional)
    if dataLocal.cb_show_trail.Value == 1
        dataLocal.trail_pts = [dataLocal.trail_pts; ee];
        if size(dataLocal.trail_pts, 1) > dataLocal.trail_maxlen
            dataLocal.trail_pts = dataLocal.trail_pts(end-dataLocal.trail_maxlen+1:end, :);
        end
        dataLocal.ee_trail.XData = dataLocal.trail_pts(:,1);
        dataLocal.ee_trail.YData = dataLocal.trail_pts(:,2);
        dataLocal.ee_trail.ZData = dataLocal.trail_pts(:,3);
    else
        % wenn aus, dann nicht weiter "füllen"
        dataLocal.ee_trail.XData = nan;
        dataLocal.ee_trail.YData = nan;
        dataLocal.ee_trail.ZData = nan;
    end

    % Diagnostics
    J = jacobian_geometric_from_fk(o_all, fk_z_axes_from_T(T_all));
    s = svd(J);
    sigmaMin = min(s);
    condJ = cond(J);

    % Live-linked velocity figure: influence + response for chosen qdot
    try
        if isfield(dataLocal, 'velBar') && isgraphics(dataLocal.velBar)
            Jv = J(1:3,:);
            infl = vecnorm(Jv, 2, 1);
            if numel(dataLocal.velBar.YData) ~= numel(infl)
                % Rebuild the velocity figure if joint count changed
                guidata(figLocal, dataLocal);
                rebuildJointAndRobotUI(figLocal, size(J,2));
                dataLocal = guidata(figLocal);
            else
                dataLocal.velBar.YData = infl;
                if isfield(dataLocal, 'velAx') && isgraphics(dataLocal.velAx)
                    dataLocal.velAx.XLim = [0.5 max(1.5, numel(infl)+0.5)];
                end
            end

            if isfield(dataLocal, 'velTable') && isgraphics(dataLocal.velTable) && isfield(dataLocal, 'velText') && isgraphics(dataLocal.velText)
                qdot = dataLocal.velTable.Data;
                if iscell(qdot)
                    qdot = cell2mat(qdot);
                end
                qdot = qdot(:);
                if numel(qdot) == size(J,2)
                    v = J(1:3,:) * qdot;
                    omega = J(4:6,:) * qdot;
                    dataLocal.velText.String = sprintf('v = [%.3g  %.3g  %.3g] m/s\n|v| = %.3g\nomega = [%.3g  %.3g  %.3g] rad/s\n|omega| = %.3g', ...
                        v(1), v(2), v(3), norm(v), omega(1), omega(2), omega(3), norm(omega));
                end
            end
        end
    catch
        % keep main UI responsive even if the linked figure is closed
    end

    % Update singular value plot (log scale)
    ss = max(s(:).', 1e-12);
    if isfield(dataLocal, 'barSing') && isgraphics(dataLocal.barSing)
        try
            if numel(dataLocal.barSing.YData) ~= numel(ss)
                cla(dataLocal.diagAx);
                grid(dataLocal.diagAx, 'on');
                title(dataLocal.diagAx, 'Singular values of J');
                xlabel(dataLocal.diagAx, 'i');
                ylabel(dataLocal.diagAx, '\sigma');
                set(dataLocal.diagAx, 'YScale', 'log');
                dataLocal.barSing = bar(dataLocal.diagAx, 1:numel(ss), ss);
                dataLocal.barSing.FaceColor = [0.25 0.25 0.25];
                dataLocal.barSing.EdgeColor = 'none';
            else
                dataLocal.barSing.YData = ss;
            end
        catch
            % fallback
            dataLocal.barSing.YData = ss;
        end
        ymin = max(min(ss) * 0.5, 1e-6);
        ymax = max(max(ss) * 2.0, 1e-3);
        dataLocal.diagAx.YLim = [ymin ymax];
        dataLocal.diagAx.XLim = [0.5 max(1.5, numel(ss)+0.5)];
    end

    % Teaching aid: separate arm/wrist 3x3 blocks (typical for 6R with wrist)
    % These are not perfect for every robot, but useful for intuition.
    sigmaArm = NaN;
    sigmaWrist = NaN;
    if size(J,2) == 6
        Jarm = J(1:3, 1:3);
        Jwrist = J(4:6, 4:6);
        sigmaArm = min(svd(Jarm));
        sigmaWrist = min(svd(Jwrist));
    end

    % Velocity ellipsoid (optional): v = Jv qdot, ||qdot||=1
    if dataLocal.cb_show_ell.Value == 1
        Jv = J(1:3, :);
        [Xe, Ye, Ze] = velocity_ellipsoid_mesh(Jv, ee, 0.08);
        dataLocal.ell.XData = Xe;
        dataLocal.ell.YData = Ye;
        dataLocal.ell.ZData = Ze;

        % Principal axes of ellipsoid (teaching aid)
        [A0, A1, A2, A3] = velocity_ellipsoid_axes(Jv, ee, 0.08);
        if isfield(dataLocal, 'ell_axes') && numel(dataLocal.ell_axes) >= 3
            if isgraphics(dataLocal.ell_axes(1))
                dataLocal.ell_axes(1).XData = [A0(1) A1(1)];
                dataLocal.ell_axes(1).YData = [A0(2) A1(2)];
                dataLocal.ell_axes(1).ZData = [A0(3) A1(3)];
            end
            if isgraphics(dataLocal.ell_axes(2))
                dataLocal.ell_axes(2).XData = [A0(1) A2(1)];
                dataLocal.ell_axes(2).YData = [A0(2) A2(2)];
                dataLocal.ell_axes(2).ZData = [A0(3) A2(3)];
            end
            if isgraphics(dataLocal.ell_axes(3))
                dataLocal.ell_axes(3).XData = [A0(1) A3(1)];
                dataLocal.ell_axes(3).YData = [A0(2) A3(2)];
                dataLocal.ell_axes(3).ZData = [A0(3) A3(3)];
            end
        end
    else
        dataLocal.ell.XData = nan(2);
        dataLocal.ell.YData = nan(2);
        dataLocal.ell.ZData = nan(2);

        if isfield(dataLocal, 'ell_axes')
            for kk = 1:numel(dataLocal.ell_axes)
                if isgraphics(dataLocal.ell_axes(kk))
                    dataLocal.ell_axes(kk).XData = nan;
                    dataLocal.ell_axes(kk).YData = nan;
                    dataLocal.ell_axes(kk).ZData = nan;
                end
            end
        end
    end

    sigmaChar = char(963); % 'σ'

    th = dataLocal.sing_threshold;
    cls = 'regular';
    if size(J,2) == 6
        isArm = sigmaArm < th;
        isWrist = sigmaWrist < th;
        if isArm && isWrist
            cls = 'arm + wrist singularity';
        elseif isArm
            cls = 'arm singularity';
        elseif isWrist
            cls = 'wrist singularity';
        else
            cls = 'regular';
        end

        dataLocal.infoText.String = sprintf(['Pose: p=[%.3f %.3f %.3f] m\n', ...
            'J:  %s_min=%.2e, cond(J)=%.2e  (%s)\n', ...
            'Arm(1:3): %s_min=%.2e   Wrist(4:6): %s_min=%.2e\n', ...
            'Cloud: %d pts  (Highlight: %d)'], ...
            ee(1), ee(2), ee(3), sigmaChar, sigmaMin, condJ, cls, ...
            sigmaChar, sigmaArm, sigmaChar, sigmaWrist, size(dataLocal.P_sing,1), K);
    else
        dataLocal.infoText.String = sprintf(['Pose: p=[%.3f %.3f %.3f] m\n', ...
            'J:  %s_min=%.2e, cond(J)=%.2e\n', ...
            'Cloud: %d pts  (Highlight: %d)'], ...
            ee(1), ee(2), ee(3), sigmaChar, sigmaMin, condJ, size(dataLocal.P_sing,1), K);
    end

    guidata(figLocal, dataLocal);
    drawnow limitrate;
end

%% Lokale Funktionen (Seminar 1/2 Stil)

function T = dh_transform(a, alpha, d, theta)
%DH_TRANSFORM Standard-DH-Transformationsmatrix (Seminar 2 Stil).
ca = cos(alpha); sa = sin(alpha);
ct = cos(theta); st = sin(theta);
T = [ ct, -st*ca,  st*sa, a*ct; ...
      st,  ct*ca, -ct*sa, a*st; ...
       0,     sa,     ca,    d; ...
       0,      0,      0,    1];
end

function [T_all, o_all, z_all] = fk_chain_dh(a, alpha, d, theta)
%FK_CHAIN_DH Vorwärtskinematik für eine DH-Kette.
% Output:
% - T_all{i}  = Transform von Basis -> Frame i (i=1..n+1, wobei n+1 = EE)
% - o_all{i}  = Ursprung o_{i-1} (i=1 entspricht Basis-Ursprung)
% - z_all{i}  = z_{i-1} Achse im Basisframe (i=1 entspricht z0)

n = numel(theta);

T = eye(4);
T_all = cell(1, n+1);

o_all = cell(1, n+1);
z_all = cell(1, n+1);

% Basis
T_all{1} = T;
o_all{1} = T(1:3, 4);
z_all{1} = T(1:3, 3);

for i = 1:n
    Ti = dh_transform(a(i), alpha(i), d(i), theta(i));
    T = T * Ti;
    T_all{i+1} = T;
    o_all{i+1} = T(1:3, 4);
    z_all{i+1} = T(1:3, 3);
end
end

function J = jacobian_geometric_from_fk(o_all, z_all)
%JACOBIAN_GEOMETRIC_FROM_FK Geometrische Jacobi (6xn) für revolute Gelenke.
%
% Formel (revolut):
%   Jv_i = z_{i-1} x (o_n - o_{i-1})
%   Jw_i = z_{i-1}
%
% Wir erwarten:
% - o_all{1}..o_all{n+1}
% - z_all{1}..z_all{n+1}

n = numel(z_all) - 1;
J = zeros(6, n);

on = o_all{end};
for i = 1:n
    zi = z_all{i};
    oi = o_all{i};
    Jv = cross(zi, (on - oi));
    Jw = zi;
    J(:, i) = [Jv; Jw];
end
end

function z_all = fk_z_axes_from_T(T_all)
%FK_Z_AXES_FROM_T Hilfsfunktion: z-Achsen aus Transformationskette.
% (Nur für Diagnose in updateScene)

m = numel(T_all);
z_all = cell(1, m);
for i = 1:m
    z_all{i} = T_all{i}(1:3, 3);
end
end

function y = wrapToPi_local(x)
%WRAPTOPI_LOCAL Wrap Winkel auf [-pi, pi] ohne Toolboxes.
% Äquivalent zu wrapToPi: y = mod(x+pi, 2*pi) - pi, aber numerisch stabil.
y = atan2(sin(x), cos(x));
end

function s = onoff(v)
%ONOFF Helper für Visible.
if v == 1
    s = 'on';
else
    s = 'off';
end
end

function [P_out, c_out] = subsample_points(P_in, c_in, fraction)
%SUBSAMPLE_POINTS Zufällige Unterstichprobe (für schnelle Interaktion).
if isempty(P_in)
    P_out = zeros(0,3);
    c_out = zeros(0,1);
    return;
end

fraction = max(min(fraction, 1), 0.01);
n = size(P_in, 1);
k = max(1, round(fraction * n));
idx = randperm(n, k);
P_out = P_in(idx, :);
c_out = c_in(idx, :);
end

function [X, Y, Z] = velocity_ellipsoid_mesh(Jv, center, scale)
%VELOCITY_ELLIPSOID_MESH Mesh für Geschwindigkeit-Ellipsoid.
% v = Jv qdot, ||qdot||=1  => v liegt in Ellipsoid, dessen Halbachsen durch
% Singularwerte von Jv bestimmt sind.
%
% Wir plotten ein skaliertes Ellipsoid, damit es optisch nicht "alles" verdeckt.

% Unit sphere mesh
[xs, ys, zs] = sphere(28);
S = [xs(:) ys(:) zs(:)].';

% A = Jv Jv^T (3x3, symmetrisch)
A = Jv * Jv.';

% Numerisch robust: eigen / svd
[U, D] = svd(A);
sig = sqrt(max(diag(D), 0));

T = U * diag(sig) * scale;
E = (T * S).';

X = reshape(E(:,1) + center(1), size(xs));
Y = reshape(E(:,2) + center(2), size(ys));
Z = reshape(E(:,3) + center(3), size(zs));
end

function [c, p1, p2, p3] = velocity_ellipsoid_axes(Jv, center, scale)
%VELOCITY_ELLIPSOID_AXES Principal axes endpoints for the velocity ellipsoid.
% Uses A = Jv*Jv' and its SVD. Axis lengths are proportional to singular
% values of Jv (sqrt of eigenvalues of A), scaled by "scale".

c = center(:).';
A = Jv * Jv.';
[U, D] = svd(A);
sig = sqrt(max(diag(D), 0));

% Guard against pathological cases
sig = max(sig, 0);

% Endpoints (from center to +axis direction)
p1 = c + (U(:,1) * sig(1) * scale).';
p2 = c + (U(:,2) * sig(2) * scale).';
p3 = c + (U(:,3) * sig(3) * scale).';
end
