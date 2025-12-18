function s = strapdown_step(s, meas, dt, cfg)
% One strapdown step.
% State uses q_WB (World <- Body). Measurements in Body.

if dt <= 0 || dt > cfg.dtMax
    s.t = s.t + max(0, dt);
    return;
end

omega = meas.gyro(:)  - s.bg;   % rad/s
f_b   = meas.accel(:) - s.ba;   % m/s^2, specific force

% 1) orientation propagation
dq = imu.models.quat_expmap(omega*dt); % integrates rotation
s.q_WB = imu.models.quat_normalize( imu.models.quat_mul(s.q_WB, dq) );

% 2) specific force to world
R_WB = imu.models.quat_to_rotm(s.q_WB);
f_W = R_WB * f_b;

% 3) world acceleration
a_W = f_W + s.g_W;

% 4) integrate (semi-implicit)
s.v_W = s.v_W + a_W*dt;
s.p_W = s.p_W + s.v_W*dt;

s.t = s.t + dt;
end
