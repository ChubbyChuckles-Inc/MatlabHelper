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

theta_offset = zeros(1,6);

% Gelenkgrenzen (rad) – bewusst moderat gewählt (typisch: +-170° etc.)
q_min = deg2rad([-170, -120, -170, -190, -120, -360]);
q_max = deg2rad([ 170,  120,  170,  190,  120,  360]);

% Startkonfiguration
q0 = deg2rad([0, -30, 60, 0, 30, 0]);

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

Q = zeros(N_samples, 6);
P = zeros(N_samples, 3);
SigmaMin = zeros(N_samples, 1);

for k = 1:N_samples
    q = q_min + rand(1,6).*(q_max - q_min);
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
link_colors = lines(6);
robot_links = gobjects(1,6);
for i = 1:6
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
panelCtrl = uipanel('Parent', fig, 'Title', 'Controls', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.60 0.28 0.36]);
panelJoints = uipanel('Parent', fig, 'Title', 'Joint Angles (deg)', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.30 0.28 0.28]);
panelDiag = uipanel('Parent', fig, 'Title', 'Diagnostics', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.70 0.08 0.28 0.20]);

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
    'Position', [0.25 0.08 0.70 0.06], 'String', {'Home', 'Wrist singular (q5=0)', 'Elbow stretched (q2=0,q3=0)'}, ...
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
barSing = bar(diagAx, 1:6, ones(1,6));
barSing.FaceColor = [0.25 0.25 0.25];
barSing.EdgeColor = 'none';
diagAx.XLim = [0.5 6.5];
diagAx.YLim = [1e-6 1e2];

% Data in guidata
data = struct();
data.a = a; data.alpha = alpha; data.d = d; data.theta_offset = theta_offset;
data.q_min = q_min; data.q_max = q_max;
data.ax = ax;
data.sc_sing = sc_sing;
data.sc_focus = sc_focus;
data.robot_links = robot_links;
data.robot_joints = robot_joints;
data.robot_ee = robot_ee;
data.ee_trail = ee_trail;
data.ell = ell;
data.ell_axes = ell_axes;

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
sliders = gobjects(1,6);
edits = gobjects(1,6);
labels = gobjects(1,6);

% Initial values
q_current = q0;

for i = 1:6
    % Joint-Controls: eine Zeile pro Gelenk (in eigenem Panel)
    y = 0.83 - (i-1)*0.155;

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
data.infoText = infoText;

data.th_slider = th_slider;
data.th_label = th_label;
data.den_slider = den_slider;
data.den_label = den_label;
data.hl_slider = hl_slider;
data.hl_label = hl_label;
data.highlightK = round(hl_slider.Value);
data.preset_menu = preset_menu;
data.diagAx = diagAx;
data.barSing = barSing;
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
    qhome = deg2rad([0, -30, 60, 0, 30, 0]);
    for ii = 1:6
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
    q = dataLocal.q_min + rand(1,6).*(dataLocal.q_max - dataLocal.q_min);
    for ii = 1:6
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

    switch v
        case 1 % Home
            q = deg2rad([0, -30, 60, 0, 30, 0]);
        case 2 % Wrist singularity (klassisch: q5 -> 0)
            q = deg2rad([0, -30, 60, 0, 0, 0]);
        case 3 % Elbow stretched (vereinfacht: q2,q3 -> 0)
            q = deg2rad([0, 0, 0, 0, 30, 0]);
        otherwise
            q = deg2rad([0, -30, 60, 0, 30, 0]);
    end

    for ii = 1:6
        q(ii) = min(max(q(ii), dataLocal.q_min(ii)), dataLocal.q_max(ii));
        dataLocal.sliders(ii).Value = q(ii);
        dataLocal.edits(ii).String = sprintf('%.1f', rad2deg(q(ii)));
    end

    guidata(figLocal, dataLocal);
    updateScene(figLocal);
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

%% Scene update
function updateScene(figLocal)
    dataLocal = guidata(figLocal);

    % Ensure cloud labels are up-to-date
    if isempty(dataLocal.th_label.String) || isempty(dataLocal.den_label.String)
        updateCloud(figLocal);
        dataLocal = guidata(figLocal);
    end

    % Read q from sliders
    q = zeros(1,6);
    for ii = 1:6
        q(ii) = dataLocal.sliders(ii).Value;
    end

    % FK chain for visualization
    [T_all, o_all, ~] = fk_chain_dh(dataLocal.a, dataLocal.alpha, dataLocal.d, q + dataLocal.theta_offset);

    % Joint positions
    pts = zeros(7,3);
    for j = 1:7
        pts(j,:) = o_all{j}.';
    end

    % Update robot plot
    for i = 1:6
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

    % Update singular value plot (log scale)
    ss = max(s(:).', 1e-12);
    if isfield(dataLocal, 'barSing') && isgraphics(dataLocal.barSing)
        dataLocal.barSing.YData = ss;
        ymin = max(min(ss) * 0.5, 1e-6);
        ymax = max(max(ss) * 2.0, 1e-3);
        dataLocal.diagAx.YLim = [ymin ymax];
    end

    % Teaching aid: separate arm/wrist 3x3 blocks (typical for 6R with wrist)
    % These are not perfect for every robot, but useful for intuition.
    Jarm = J(1:3, 1:3);
    Jwrist = J(4:6, 4:6);
    sigmaArm = min(svd(Jarm));
    sigmaWrist = min(svd(Jwrist));

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
