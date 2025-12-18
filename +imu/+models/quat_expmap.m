function dq = quat_expmap(phi)
% phi: 3x1 rotation vector (rad), returns quaternion Exp(phi/2) implicitly
phi = phi(:);
ang = norm(phi);
if ang < 1e-12
    dq = [1; 0.5*phi]; % small-angle approx
    dq = imu.models.quat_normalize(dq);
    return;
end
axis = phi/ang;
half = ang/2;
dq = [cos(half); axis*sin(half)];
dq = imu.models.quat_normalize(dq);
end
