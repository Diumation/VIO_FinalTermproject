function [init, dbg] = align_initial(imuData, cfg)
t = imuData.t;
a = imuData.accel;
w = imuData.gyro;

stationary = imu.frontend.detect_stationary(imuData, cfg);

idx0 = find(stationary, 1, "first");
if isempty(idx0), idx0 = 1; end

fs = imuData.meta.fs;
winN = max(5, round(cfg.init.windowSec * fs));
i1 = idx0;
i2 = min(numel(t), idx0 + winN - 1);

a0 = mean(a(i1:i2,:), 1).';
w0 = mean(w(i1:i2,:), 1).';

% World gravity: camera-style (x-right, y-down, z-forward)
g_W = [0; cfg.g; 0];
ghat_W = g_W / norm(g_W);

u_b = a0 / max(1e-9, norm(a0));

% Quaternion to rotate u_b -> ghat_W
q_WB_rp = quat_from_two_unit_vectors(u_b, ghat_W);

% yaw0 about world y-axis (down)
q_yaw = quat_axis_angle([0;1;0], cfg.init.yaw0);

q_WB0 = imu.models.quat_normalize( imu.models.quat_mul(q_yaw, q_WB_rp) );

bg0 = zeros(3,1);
ba0 = zeros(3,1);

if cfg.bias.estimateFromStationary && any(stationary)
    bg0 = w0;

    R_WB = imu.models.quat_to_rotm(q_WB0);
    R_BW = R_WB.';
    f_b_expected = -R_BW * g_W;
    ba0 = a0 - f_b_expected;
end

init.idx0  = i1;
init.t0    = t(i1);
init.q_WB0 = q_WB0;
init.bg0   = bg0;
init.ba0   = ba0;
init.g_W   = g_W;

dbg.stationaryFound = any(stationary);
dbg.idx0 = idx0;
dbg.a0 = a0;
dbg.w0 = w0;
dbg.q_WB0 = q_WB0;
dbg.g_W = g_W;

end

function q = quat_from_two_unit_vectors(u, v)
dotuv = max(-1, min(1, dot(u,v)));
if dotuv > 1 - 1e-8
    q = [1;0;0;0]; return;
end
if dotuv < -1 + 1e-8
    axis = cross(u, [1;0;0]);
    if norm(axis) < 1e-6
        axis = cross(u, [0;0;1]);
    end
    axis = axis / norm(axis);
    q = quat_axis_angle(axis, pi);
    return;
end
axis = cross(u, v);
s = sqrt((1+dotuv)*2);
q = [s/2; axis(:)/s];
q = imu.models.quat_normalize(q);
end

function q = quat_axis_angle(axis, angle)
axis = axis(:);
axis = axis / max(1e-12, norm(axis));
half = angle/2;
q = [cos(half); axis*sin(half)];
end
