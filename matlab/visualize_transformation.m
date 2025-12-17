function ax = visualize_transformation(T, varargin)
%VISUALIZE_TRANSFORMATION Plot base and transformed frames for a 4x4 transform.
%   visualize_transformation(T) creates a new figure and plots the base frame
%   at the origin plus the transformed frame defined by T.
%
%   visualize_transformation(T, ax) plots into an existing axes.
%   visualize_transformation(T, titleText) uses a custom title.
%   visualize_transformation(T, ax, titleText) uses both.
%
%   Returns the axes handle used.

% Validate
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
    fig = figure('Name', plot_title, 'NumberTitle', 'off');
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
R = T(1:3, 1:3);

axis_colors = {'r', 'g', 'b'};
I3 = eye(3);

for axis_idx = 1:3
    dir = scale * I3(:, axis_idx);
    q = quiver3(ax, origin_base(1), origin_base(2), origin_base(3), ...
        dir(1), dir(2), dir(3), 0, 'Color', axis_colors{axis_idx}, ...
        'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    q.HandleVisibility = 'off';
end

for axis_idx = 1:3
    dir = scale * R(:, axis_idx);
    q = quiver3(ax, origin_target(1), origin_target(2), origin_target(3), ...
        dir(1), dir(2), dir(3), 0, 'Color', axis_colors{axis_idx}, ...
        'LineWidth', 2.2, 'MaxHeadSize', 0.6, 'LineStyle', '--');
    q.HandleVisibility = 'off';
end

l = plot3(ax, [origin_base(1), origin_target(1)], ...
           [origin_base(2), origin_target(2)], ...
           [origin_base(3), origin_target(3)], 'k:', 'LineWidth', 1.2);
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
