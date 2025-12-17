function T = rpy2rotm(rpy, verbose)
%RPY2ROTM Roll-Pitch-Yaw (ZYX) to homogeneous rotation.
%   T = rpy2rotm([roll pitch yaw]) returns a 4x4 transform with rotation.
%   This matches the convention used in your seminar scripts: R = Rz(yaw)*Ry(pitch)*Rx(roll).

if nargin < 2
    verbose = false;
end

roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);

T = rotZ(yaw) * rotY(pitch) * rotX(roll);

if verbose
    fprintf('rpy2rotm -> yaw=%.3f, pitch=%.3f, roll=%.3f rad.\n', yaw, pitch, roll);
end
end
