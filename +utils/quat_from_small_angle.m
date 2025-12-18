function dq = quat_from_small_angle(dth)
% small angle approx
dq = [1; 0.5*dth(:)];
dq = dq / norm(dq);
end
