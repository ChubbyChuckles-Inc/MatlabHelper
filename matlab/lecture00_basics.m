% Vorlesung 0 — MATLAB‑Grundlagen für Robotik (Mathematik, Visualisierung, Kinematik)
%
% Ausführung
%   - Datei in MATLAB öffnen und „Run“ ausführen (oder abschnittsweise mit %%‑Zellen)
%   - Arbeitsordner auf das Repo bzw. den Ordner „matlab“ setzen
%   - Abbildungen geöffnet lassen, wenn Sie in ein PDF exportieren
%
% Lernziele
%   1) MATLAB‑Mathe‑Basics (Vektoren/Matrizen, Lineare Algebra, Numerik)
%   2) Visualisierung: 2D/3D‑Plots, Flächen, Koordinatenrahmen
%   3) Kinematik: Rotationsmatrizen, homogene Transformationen, einfache Ketten
%   4) DH‑basierte Vorwärtskinematik (kurz) inkl. Rahmendarstellung
%
% Konventionen
%   - Winkel sind in Radiant, außer wenn explizit in Grad ausgegeben
%   - Rechtshändiges Koordinatensystem; Z‑Achse zeigt standardmäßig „aus der Seite“
%   - Transformation T bildet mittels Post‑Multiplikation von einer Quelle auf ein Ziel ab
%   - R ist 3×3‑Rotation, p ist 3×1‑Position, T ist 4×4‑Homogene

%% Hilfsfunktionen (werden zuerst definiert und anschließend genutzt)

% d2r: Umrechnung von Grad nach Radiant
%   Eingabe: deg (Skalar/Array) in Grad
%   Ausgabe: Radiant gleicher Größe
function out = d2r(deg)
    out = deg * pi/180;
end

% r2d: Umrechnung von Radiant nach Grad
%   Eingabe: rad (Skalar/Array) in Radiant
%   Ausgabe: Grad gleicher Größe
function out = r2d(rad)
    out = rad * 180/pi;
end

% rotX3/rotY3/rotZ3: elementare 3×3‑Rotationsmatrizen
%   Jeweils eine Rechtsdrehung um die entsprechende Achse.
%   Eigenschaften: R^T R = I, det(R) = 1 (SO(3)).
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

% makeT: zusammengesetzte 4×4‑Transformation aus R und p
%   T = [ R p; 0 0 0 1 ]
%   R: 3×3, p: 3×1
function T = makeT(R, p)
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = p(:);
end

% transl4: reine Translation als 4×4‑Matrix
%   Praktisch zum sequentiellen Zusammensetzen von Posen.
function T = transl4(x, y, z)
    T = eye(4);
    T(1:3,4) = [x; y; z];
end

% isSO3: Prüft, ob R eine gültige Rotationsmatrix ist
%   Test: Orthonormalität und Determinante 1 innerhalb Toleranz.
function tf = isSO3(R, tol)
    if nargin < 2, tol = 1e-9; end
    tf = norm(R'*R - eye(3), 2) < tol && abs(det(R) - 1) < tol;
end

% tprint4: kompaktes Ausgeben einer 4×4‑Transformation
%   Druckt Position p und die 3×3‑Rotationsmatrix.
function tprint4(T, label)
    if nargin < 2, label = 'Transformation'; end
    fprintf('\n%s\n', label);
    fprintf('  p (m): [%.4f %.4f %.4f]\n', T(1,4), T(2,4), T(3,4));
    fprintf('  R =\n');
    disp(T(1:3,1:3));
end

% draw_frame3: zeichnet einen Koordinatenrahmen in 3D
%   Achsenfarben: X=rot, Y=grün, Z=blau; Pfeillänge über scale.
function draw_frame3(ax, T, scale, lw)
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

% dh: Standard‑Denavit‑Hartenberg‑Matrix
%   Parameter: a (Linklänge), alpha (Verdrehung), d (Verschiebung), theta (Gelenkwinkel)
%   Liefert die homogene Einzel‑Transformation eines Glieds.
function A = dh(a, alpha, d, theta)
    c = cos(theta); s = sin(theta); ca = cos(alpha); sa = sin(alpha);
    A = [c, -s*ca,  s*sa, a*c;
         s,  c*ca, -c*sa, a*s;
         0,     sa,    ca,   d;
         0,      0,     0,   1];
end

% fkine_dh: Vorwärtskinematik für eine DH‑Kette
%   dh_table: [a, alpha, d, theta_offset] je Zeile
%   q: Gelenkwinkelvektor (in Radiant)
%   Rückgabe: Endpose Tn und Zellarray Ts aller Zwischenposen
function [Tn, Ts] = fkine_dh(dh_table, q)
    n = size(dh_table,1);
    Ts = cell(n+1,1); Ts{1} = eye(4);
    for i = 1:n
        a = dh_table(i,1); alpha = dh_table(i,2); d = dh_table(i,3); th0 = dh_table(i,4);
        th = q(i) + th0;
        Ts{i+1} = Ts{i} * dh(a, alpha, d, th);
    end
    Tn = Ts{end};
end

% plot_planar_chain: 2D‑Darstellung einer Gelenkkette (Punktfolge)
%   points: 2×N mit den Gelenkpositionen in der Ebene
function plot_planar_chain(ax, points, opt)
    if nargin < 3, opt = struct(); end
    if ~isfield(opt,'lw'), opt.lw = 3; end
    if ~isfield(opt,'ms'), opt.ms = 30; end
    plot(ax, points(1,:), points(2,:), '-o', 'LineWidth', opt.lw, 'MarkerSize', opt.ms/3);
    axis(ax, 'equal'); grid(ax, 'on');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
end

%% 1) Einstieg: Aufräumen und Initialisierung
clc;  % Räumt das Command Window auf
rng(0);  % Setzt den Zufallszahlengenerator für reproduzierbare Ergebnisse
fprintf('Vorlesung 0: MATLAB‑Grundlagen für Robotik\n');  % Gibt den Titel im Command Window aus
datum = datestr(now);  % Wandelt die aktuelle Systemzeit in eine formatierte Zeichenkette um
fprintf('Datum: %s\n', datum);  % Zeigt das aktuelle Datum an

%% 2) Mathematik-Grundlagen: Felder und Lineare Algebra
fprintf('\n=== Mathematik-Grundlagen ===\n');

% Felder und elementweise Operationen
v = (1:5);                   % Zeilenvektor
w = v.^2;                    % Elementweise Potenz
A = [1 2; 3 4];              % 2x2-Matrix
B = [2 -1; 1 0.5];
C = A*B;                     % Matrixprodukt
D = A.*B;                    % Elementweise Multiplikation
fprintf('A*B (Matrixprodukt) =\n'); disp(C);  % disp zeigt die Matrix im Command Window an
fprintf('A.*B (elementweise) =\n'); disp(D);

% Indizierung und Teilbereiche
M = reshape(1:12, [3,4]);    % reshape formt den Zahlenbereich zu einer 3x4-Matrix um
col_text = num2str(M(:,2).');  % num2str wandelt den Spaltenvektor in eine Zeichenkette um
fprintf('Indexierung: M(2,3) = %d, M(:,2) = [ %s ]\n', M(2,3), col_text);

% Implizite Erweiterung (Broadcasting)
row = [1 2 3]; col = [10; 20; 30];
S = row + col;               % 3x3 durch Broadcasting
fprintf('Broadcasting (Zeile + Spalte) =\n'); disp(S);

% Lineare Gleichungssysteme und Konditionierung
A = [4 1 0; 1 3 -1; 0 -1 2]; b = [1; 2; 0.5];
x = A\b; res = norm(A*x - b);  % norm misst die Abweichung zwischen Ax und b
fprintf('Löse Ax=b mit \\: ‖Ax-b‖ = %.2e\n', res);
fprintf('cond_2(A) = %.2f (Konditionszahl)\n', cond(A));  % cond gibt die Konditionszahl der Matrix aus

% SVD und Ausgleichsrechnung
M = randn(5,3);  % randn erzeugt normalverteilte Zufallszahlen mit Mittelwert 0
y = randn(5,1);
[U,S,V] = svd(M, 'econ'); %#ok<ASGLU>  % svd berechnet die Singulärwertzerlegung der Matrix
x_ls = M\y; sig = diag(S).';  % diag extrahiert die Diagonale der Singulärwertmatrix
fprintf('SVD(M): Singulärwerte = [%.3f %.3f %.3f]\n', sig);
fprintf('Least‑Squares‑Lösung: ‖x‖ = %.3f\n', norm(x_ls));||

%% 3) Visualisierung: 2D und 3D
fprintf('\n=== Visualisierung — Grundlagen ===\n');

% 2D-Darstellung
t = linspace(0, 2*pi, 200);  % linspace erzeugt 200 äquidistante Stützstellen zwischen 0 und 2π
y1 = sin(t);  % sin berechnet die Sinuswerte für die Zeitbasis
y2 = cos(t);  % cos liefert die Cosinuswerte zur Vergleichskurve
figure('Name','2D‑Plots');  % figure öffnet ein neues Grafikfenster mit der angegebenen Beschriftung
subplot(1,2,1);  % subplot(1,2,1) teilt das Fenster in 1x2 Achsen und aktiviert die linke Achse
plot(t, y1, 'b-', 'LineWidth', 1.5);  % plot zeichnet den Funktionsverlauf als Linie
hold on;  % hold on behält die aktuelle Achse für weitere Plots aktiv
plot(t, y2, 'r--', 'LineWidth', 1.5);
grid on;  % grid on blendet ein Koordinatengitter ein
xlabel('t (rad)');  % xlabel setzt die Beschriftung der x-Achse
ylabel('Amplitude');  % ylabel setzt die Beschriftung der y-Achse
legend('sin(t)', 'cos(t)');  % legend erzeugt eine Legende für die dargestellten Kurven
title('Sinus und Cosinus');  % title vergibt einen Achsentitel

% Streudiagramm mit Beschriftung
xdata = linspace(-1,1,20);
noise = 0.05*randn(size(xdata));  % size liefert die Dimensionen von xdata für das Rauschsampling
ydata = xdata.^2 + noise;
subplot(1,2,2);
scatter(xdata, ydata, 40, 'filled');  % scatter stellt diskrete Punkte mit Markierungen dar
grid on; xlabel('x'); ylabel('y'); title('Punktwolke');

% 3D-Darstellung
figure('Name','3D‑Plots');
subplot(1,2,1);
t = linspace(0, 4*pi, 200); r = 0.2; z = linspace(0, 1, 200);
x = r*cos(t); y = r*sin(t);
plot3(x, y, z, 'm-', 'LineWidth', 2);  % plot3 zeichnet eine räumliche Kurve
grid on; axis equal;  % axis equal erzwingt gleiche Skalierung in allen Achsen
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D‑Helix');

subplot(1,2,2);
[X,Y] = meshgrid(linspace(-1,1,50));  % meshgrid erzeugt ein Gitter aus x/y-Koordinaten
Z = exp(-3*(X.^2 + Y.^2));  % exp wertet den Exponentialausdruck für das Höhenprofil aus
surf(X,Y,Z);  % surf stellt eine farbige Oberfläche dar
shading interp;  % shading interp glättet die Farbverläufe zwischen den Facetten
colormap turbo;  % colormap legt die Farbpalette für die Grafik fest
axis tight;  % axis tight schneidet den Plot auf den Datenbereich zu
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Gauß‑Hügel (Fläche)');

%% 4) Rotationen und homogene Transformationen
fprintf('\n=== Rotationen und Transformationen ===\n');
R = rotZ3(d2r(30)) * rotY3(d2r(15)) * rotX3(d2r(-10));
fprintf('isSO3(R) = %d, det(R) = %.6f\n', isSO3(R), det(R));
p = [0.3; -0.1; 0.2]; T = makeT(R, p);
tprint4(T, 'Beispiel‑Transformation T');

% Transformationen kombinieren: Basis->A->B
Ta = makeT(rotZ3(d2r(20)), [0.2; 0.0; 0.0]);
Tb = makeT(rotY3(d2r(10)), [0.0; 0.1; 0.0]);
T_ab = Ta * Tb; tprint4(T_ab, 'Komposition T = Ta * Tb');

%% 5) Koordinatenrahmen in 3D visualisieren
fprintf('\n=== Koordinatenrahmen visualisieren ===\n');
fig = figure('Name','Rahmen');
ax = axes(fig);  % axes erzeugt ein neues Achsenobjekt im angegebenen Figure
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 45, 25);  % view legt die Blickrichtung auf die 3D-Achse fest
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z'); title(ax, 'Basis‑ und rotiertes Koordinatensystem');
draw_frame3(ax, eye(4), 0.1, 2);          % Basisrahmen
draw_frame3(ax, T, 0.1, 2);               % transformierter Rahmen
% Hinweis: Farbkonvention der Achsen: X=rot, Y=grün, Z=blau

%% 6) Planare 2R-Kette ohne DH-Parameter
fprintf('\n=== Planare 2R‑Vorwärtskinematik (Geometrie) ===\n');
l1 = 0.35; l2 = 0.25; q = [d2r(35), d2r(-25)];
p0 = [0;0];
p1 = p0 + [l1*cos(q(1)); l1*sin(q(1))];
p2 = p1 + [l2*cos(q(1)+q(2)); l2*sin(q(1)+q(2))];
fprintf('Endeffektor (x,y) = [%.3f, %.3f] m\n', p2(1), p2(2));

fig2 = figure('Name','Planar 2R'); ax2 = axes(fig2);
plot_planar_chain(ax2, [p0 p1 p2]);  % plot_planar_chain zeichnet die Gelenkkette als Linienzug
title(ax2, '2R‑Planararm');
label_text = sprintf('  EE [%.2f, %.2f] m', p2(1), p2(2));  % sprintf formatiert den Text mit Zahlenwerten
text(ax2, p2(1), p2(2), label_text);  % text platziert eine Beschriftung an den angegebenen Koordinaten

%% 7) DH-basierte Vorwärtskinematik mit Rahmendarstellung
fprintf('\n=== DH‑basierte Vorwärtskinematik ===\n');
dh_table = [
    l1, 0, 0, 0;  % a, alpha, d, theta_offset
    l2, 0, 0, 0
];
[Tee, Ts] = fkine_dh(dh_table, q);  % fkine_dh berechnet die Vorwärtskinematik der DH-Kette
tprint4(Tee, 'T_{Basis->EE} aus DH‑Kette');

% Zwischenrahmen darstellen
fig3 = figure('Name','DH‑Rahmen'); ax3 = axes(fig3);
hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal'); view(ax3, 30, 25);
xlabel(ax3, 'X'); ylabel(ax3, 'Y'); zlabel(ax3, 'Z'); title(ax3, 'Rahmen entlang der DH‑Kette');
for i = 1:numel(Ts)
    draw_frame3(ax3, Ts{i}, 0.08, 2);
end
draw_frame3(ax3, Tee, 0.1, 3);

fprintf('\nEnde von Vorlesung 0. Vorschlag: Fahren Sie mit Vorlesung 1 (Utilities) fort.\n');
