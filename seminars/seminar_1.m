% 1. Seminar - Einführung Grundlagen der Robotik
% 
% MATLAB Shortcuts
% 
% - Strg + N - Neues Dokument
% - Strg + S - Dokument Speichern
% - Strg + R - Kommentieren
% - Strg + 0 - Command Window
% - Strg + C - Programm stoppen
% - F5 - Ausführen
% 
% Command Befehle
% 
% - clc = Löscht Inhalt Command Window
% - clearvars = Löschen von Variablen

clearvars;
clc;

fprintf("Hello World.")

Z = zeros(2, 4) % 2 Zeilen | 4 Spalten Matrix mit nur Nullen
Z = zeros(3) % 3x3 matrix aus Nullen

A = ones(4) % 4x4 Matrix mit Einsen

T = eye(5) % 5x5 Einheitsmatrix

R = rand(4, 1) % Vektor mit 4 Zeilen
R = rand(1, 4) % Zeilenmatrix mit 4 Spalten

v = 1:5 % Zeilenvektor mit Zahlen von 1 bis 5
v = 1:0.5:3 % Zahlen von 1 bis 3 mit Abstand 0.5
v = linspace(0, 10, 12) % Zeilenvektor mit Werten von 0 bis 10 in 12 Schritten
v = logspace(10, 15, 6) % logarithische Einteilung von 10^10 bis 10^15 in 6 Abschnitten
v = logspace(log10(10), log10(15), 6)

% Mehrdimensionale Matrizen (Arrays)
mdm = zeros(3,3,3,3,3,3,3)

clearvars; % Löscht variablen wie T und A und R
clc; % Löscht den Bildschirm

% Funktion für Rotation um X-Achse

function T = rotX(theta)
T = eye(4);
T(2:3,2:3) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
fprintf('rotX(%.3f rad) generiert.\n', theta);
end

rx = rotX(pi/6)

% Funktion die Transformation formatiert ausgibt
function tprint(T, label)
if nargin < 2, label = 'Transformation'; end
fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f] m \n', T(1:3, 4));
fprintf('Rotation: (3x3 oben-links):\n');
disp(T(1:3, 1:3));
end

tprint(rx);
tprint(rx, 'Rotation um X (30 deg)');

clc;

% Funktion für Rotation um Y-Achse
function T = rotY(theta)
T = eye(4);
T([1 3], [1 3]) = [cos(theta), sin(theta); -sin(theta), cos(theta)];
fprintf('rotY(%.3f rad) generiert.\n', theta);
end

% Funktion für Rotation um Z-Achse
function T = rotZ(theta)
T = eye(4);
T(1:2, 1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
fprintf('rotZ(%0.3f rad) generiert.\n', theta);
end

%% 1. Allgemeine Rotations- und Translationsoperationen
fprintf('\n === Schritt 1: Allgemeine Rotations- und Translationsoperationen === \n')
% Rotation um x-Achse um 18 Grad
% Rotation um y-Achse um 15 Grad
% Rotation um y-Achse um -22.5 Grad

rx_1 = rotX(pi/10)
tprint(rx_1, 'Rotation um X (18 Grad)')
ry_1 = rotY(pi/12)
tprint(ry_1, 'Rotation um Y (15 Grad)')
rz_1 = rotZ(-pi/8)
tprint(rz_1, 'Rotation um Z (-22.5 Grad)')

%Funktion für Translation in x-, y- und z-Richtung
function T = transl(x, y, z)
T = eye(4);
T(1:3, 4) = [x; y; z];
fprintf('transl([%.3f %.3f %.3f]) generiert.\n', x, y, z);
end

%% 2. Erzeugung einer Endposition mit homogenen Matrizen
fprintf('\n === Schritt 2: Erzeugung einer Basis-EE Transformation === \n');
T = transl(0.3, 0.1, 0.2) * rotZ(pi/6) * rotY(pi/12);
tprint(T, 'Basis zu EE Transformation');
visualize_transformation(T);

% Funktion für Umwandlung von Roll-Pitch-Yaw in Rotationsmatrix
function T = rpy2rotm(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
T = rotZ(yaw) * rotY(pitch) * rotX(roll);
fprintf('rpy2rotm ->  yaw=%.3f, pitch=%.3f, roll=%.3f rad.\n', yaw, pitch, roll);
end

% Funktion für Umwandlung von Rotationsmatrix in Roll-Pitch-Yaw
function rpy = rotm2rpy(T)
R = T(1:3, 1:3); %Extrahiere Rotationsmatrix
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3))); % Berechne Neigungswinkel mit atan2
if abs(cos(pitch)) < 1e-8
    warning('Kardanische Blockade: Setze yaw für Lösung 0.');
    roll = atan2(R(1,2), R(2,2));
    yaw = 0;
else
    roll = atan2(R(3,2), R(3,3));
    yaw = atan2(R(2,1), R(1,1));
end
rpy = [roll, pitch, yaw];
fprintf('rpy2rotm ->  yaw=%.3f, pitch=%.3f, roll=%.3f rad.\n', yaw, pitch, roll);
end

%% 3. Umwandlung zwischen roll-pitch-yaw und Rotationsmatrix
fprintf('\n === Schritt 3: Roll-Pitch-Yaw Umwandlung === \n');
rpy = [10, 20, 5] * pi/180; % roll, pitch, yaw in rad
tprint(rpy2rotm(rpy), 'RPY Rotation');
test = rotm2rpy(rpy2rotm(rpy));
fprintf('RPY (deg): [%.2f %.2f %.2f]\n', test * 180/pi);

%% 4. Visualisierung von Transformationen in 3D
function visualize_transformation(T, varargin)
%   Stellt Ausgangs- und Zielpose einer 4x4-Transformation dar.
%   Die Funktion nimmt eine homogene 4x4-Matrix entgegen und zeichnet zwei
%   Koordinatenrahmen im Raum:
%       1) den Ausgangsrahmen der Basis (Identität) am Ursprung
%       2) den um T transformierten Rahmen inklusive verschobenem Ursprung
%   Optional kann als weiteres Argument ein bestehendes Achsenobjekt
%   übergeben werden, sodass mehrere Transformationen in der gleichen
%   Ansicht verglichen werden können. Zusätzlich lässt sich ein eigener
%   Titel für die Visualisierung angeben.

% Validierung: stelle sicher, dass ein 4x4-Array übergeben wird.
if ~isequal(size(T), [4, 4])  % Prüft, ob T exakt eine 4x4-Matrix ist
    error('visualize_transformation:InvalidSize', ...
        'Erwarte eine 4x4-Transformationsmatrix, erhalten wurde %s.', mat2str(size(T)));  % error bricht mit Fehlermeldung ab, mat2str konvertiert die Größe in Text
end

% Optionalargumente analysieren: vorhandene Achse und/oder eigenen Titel verwenden.
ax = [];
plot_title = 'Visualisierung der Transformation';
for idx = 1:numel(varargin)
    candidate = varargin{idx};
    if isa(candidate, 'matlab.graphics.axis.Axes')  % isa testet den Datentyp des Arguments
        ax = candidate;
    elseif (ischar(candidate) && ~isempty(candidate)) || (isstring(candidate) && isscalar(candidate))
        plot_title = char(candidate);  % char wandelt string in char-Array um
    else
        error('visualize_transformation:InvalidArgument', ...
            'Zusatzargument muss Achsenobjekt oder Titeltext sein.');
    end
end

if isempty(ax)
    fig = figure('Name', plot_title, 'NumberTitle', 'off'); %#ok<NASGU>  % figure erzeugt ein neues Grafikfenster
    ax = axes('Parent', fig);  % Neue Achse explizit anlegen
end

% Achse vorbereiten: Gitter, gleiche Skalierung und eine angenehme Kameraperspektive.
hold(ax, 'on');  % hold on verhindert, dass nachfolgende Plots die Achse löschen
grid(ax, 'on');  % grid blendet ein Koordinatengitter ein
axis(ax, 'equal');  % axis equal erzwingt gleiche Skalierung auf allen Achsen
view(ax, 45, 25);  % view setzt die Kameraperspektive auf Azimut 45° und Elevation 25°
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');  % Achsenbeschriftungen setzen
title(ax, plot_title);  % Überschrift für die Grafik

% Skalierungsfaktor für die Achsenlängen der Frames.
scale = 0.15;

% Hilfsvariablen: Start- und Zielursprung, sowie Rotationsmatrix der Transformation.
origin_base = [0; 0; 0];
origin_target = T(1:3, 4);
rotation = T(1:3, 1:3);

% Farben für X-, Y- und Z-Achse entsprechend der Robotik-Konvention (rot, grün, blau).
axis_colors = {'r', 'g', 'b'};
basis_vectors = eye(3);

% Zeichne den Basisrahmen mit drei Pfeilen ausgehend vom Ursprung.
for axis_idx = 1:3
    basis_dir = scale * basis_vectors(:, axis_idx);
    q = quiver3(ax, origin_base(1), origin_base(2), origin_base(3), ...
        basis_dir(1), basis_dir(2), basis_dir(3), 0, ...
        'Color', axis_colors{axis_idx}, 'LineWidth', 1.5, 'MaxHeadSize', 0.5); %#ok<NASGU>  % quiver3 zeichnet gerichtete Pfeile im 3D-Raum
    q.HandleVisibility = 'off';
end

% Zeichne den transformierten Rahmen: Ursprung verschoben, Achsen durch Rotation gedreht.
for axis_idx = 1:3
    target_dir = scale * rotation(:, axis_idx);
    q = quiver3(ax, origin_target(1), origin_target(2), origin_target(3), ...
        target_dir(1), target_dir(2), target_dir(3), 0, ...
        'Color', axis_colors{axis_idx}, 'LineWidth', 2.2, 'MaxHeadSize', 0.6, ...
        'LineStyle', '--'); %#ok<NASGU>  % Strichelte Pfeile kennzeichnen den transformierten Frame
    q.HandleVisibility = 'off';
end

% Verbinde Ausgangs- und Zielursprung, um die reine Translation zu verdeutlichen.
l = plot3(ax, [origin_base(1), origin_target(1)], ...
           [origin_base(2), origin_target(2)], ...
           [origin_base(3), origin_target(3)], 'k:', 'LineWidth', 1.2); %#ok<NASGU>  % plot3 verbindet Punkte im Raum als Linie
l.HandleVisibility = 'off';

% Markiere die Ursprünge als Punkte, um sie klar voneinander abheben zu können.
scatter3(ax, origin_base(1), origin_base(2), origin_base(3), 40, 'filled', ...
    'MarkerFaceColor', '#1f77b4', 'DisplayName', 'Basisursprung');  % scatter3 plottet Marker im Raum
scatter3(ax, origin_target(1), origin_target(2), origin_target(3), 40, 'filled', ...
    'MarkerFaceColor', '#ff7f0e', 'DisplayName', 'Transformierter Ursprung');

% Legende einmalig anlegen. Beim erneuten Aufruf mit derselben Achse bleibt sie erhalten.
if isempty(ax.Legend)
    legend(ax, 'show', 'Location', 'bestoutside');  % legend zeigt die DisplayNames der Objekte an
end

% Zeichenvorgang abschließen, damit spätere Plots nicht unbeabsichtigt blockieren.
hold(ax, 'off');  % hold off gibt die Achse wieder für separate Plots frei
end

% Beispielaufruf: vorhandene Achse und individueller Titel werden via varargin übergeben.
demo_fig = figure('Name', 'Beispiel visualize\_transformation', 'NumberTitle', 'off'); %#ok<NASGU>
demo_ax = axes('Parent', demo_fig);  % Neue Achse explizit anlegen
visualize_transformation(T, demo_ax, 'Demonstration: Benutzerdefinierter Titel');

%% Aufgabenblock: Übungsaufgaben aus Lecture 1 (übersetzt)

%% Aufgabe 1 — Zusammensetzen einer einfachen Rotation um Z
% Ziel: rotZ verwenden, um die Welt-X-Achse um +30° zu drehen und das
% Ergebnis mit dem erwarteten Orientierungsvektor zu vergleichen.
fprintf('\n[Aufgabe 1] Schritt 1: Transformation mit rotZ erzeugen.\n');
Rz = rotZ(deg2rad(30));
tprint(Rz, 'Rotation um +Z um 30°');
fprintf('[Aufgabe 1] Schritt 2: Die Einheits-X-Achse rotieren.\n');
ex = [1; 0; 0; 1];
ex_rotated = Rz * ex;
fprintf('Rotierter X-Achsen-Vektor (homogen): [%.3f %.3f %.3f %.0f]^T\n', ex_rotated);

%% Aufgabe 2 — Punkt mit Translation und anschließender Rotation bewegen
% Ziel: Einen Punkt erst um (0.2, -0.1, 0.05) m verschieben und danach um
% -20° um die Y-Achse drehen. Sowohl Zwischen- als auch Endpositionen ausgeben.
fprintf('\n[Aufgabe 2] Schritt 1: Translation aufbauen.\n');
T_translate = transl(0.2, -0.1, 0.05);
tprint(T_translate, 'Reine Translation');
fprintf('[Aufgabe 2] Schritt 2: Rotation um Y anwenden.\n');
Ty = rotY(deg2rad(-20));
T_total = T_translate * Ty;
tprint(T_total, 'Kombinierte Transformation');
fprintf('[Aufgabe 2] Schritt 3: Punkt p = [0.1 0.0 0.05 1]^T transformieren.\n');
p = [0.1; 0.0; 0.05; 1];
p_world = T_total * p;
fprintf('Transformierter Punkt: [%.3f %.3f %.3f]^T m\n', p_world(1:3));

%% Aufgabe 3 — Validierung der RPY-Umwandlungen
% Ziel: Ein beliebiges RPY-Triple in eine Rotationsmatrix umwandeln und die
% Winkel zurückgewinnen. Den Rekonstruktionsfehler in Grad berichten.
fprintf('\n[Aufgabe 3] Schritt 1: Test-Roll/Pitch/Yaw festlegen.\n');
rpy_test = deg2rad([15, 25, -10]);
fprintf('Ausgangs-RPY (deg): [%.2f %.2f %.2f]\n', rad2deg(rpy_test));
fprintf('[Aufgabe 3] Schritt 2: In eine Rotationsmatrix überführen.\n');
R = rpy2rotm(rpy_test);
tprint(R, 'Rotation aus RPY');
fprintf('[Aufgabe 3] Schritt 3: Winkel mit rotm2rpy zurückgewinnen.\n');
rpy_recovered = rotm2rpy(R);
err = rad2deg(rpy_recovered - rpy_test);
fprintf('Zurückgewonnene RPY (deg): [%.2f %.2f %.2f]\n', rad2deg(rpy_recovered));
fprintf('Rekonstruktionsfehler (deg): ‖Δ‖₂ = %.3e\n', norm(err));