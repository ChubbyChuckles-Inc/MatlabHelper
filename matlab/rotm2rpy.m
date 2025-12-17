function rpy = rotm2rpy(T, verbose)
%ROTM2RPY Extract roll-pitch-yaw (ZYX) from rotation matrix / transform.
%   rpy = rotm2rpy(T) accepts 3x3 R or 4x4 T.
%   Returns [roll pitch yaw] in radians.

if nargin < 2
    verbose = false;
end

if isequal(size(T), [4 4])
    R = T(1:3,1:3);
elseif isequal(size(T), [3 3])
    R = T;
else
    error('rotm2rpy:InvalidSize', 'Expected 3x3 or 4x4 input, got %s.', mat2str(size(T)));
end

pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));

if abs(cos(pitch)) < 1e-8
    % Gimbal lock: yaw becomes ambiguous
    warning('rotm2rpy:GimbalLock', 'Gimbal lock: pitch near ±90°, using fallback.');
    roll = atan2(R(1,2), R(2,2));
    yaw = 0;
else
    roll = atan2(R(3,2), R(3,3));
    yaw  = atan2(R(2,1), R(1,1));
end

rpy = [roll, pitch, yaw];

if verbose
    fprintf('rotm2rpy -> roll=%.3f, pitch=%.3f, yaw=%.3f rad.\n', roll, pitch, yaw);
end
end
