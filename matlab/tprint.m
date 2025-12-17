function tprint(T, label)
%TPRINT Pretty-print a homogeneous transform.
%   tprint(T) prints position and rotation.
%   tprint(T,label) uses a custom label.

if nargin < 2
    label = 'Transform';
end

fprintf('\n%s\n', label);
fprintf('Position: [%.3f %.3f %.3f] m\n', T(1:3,4));
fprintf('Rotation (top-left 3x3):\n');
disp(T(1:3,1:3));
end
