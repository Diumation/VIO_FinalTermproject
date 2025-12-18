function [traj, dbg] = integrate_zupt(imuDataC, state0, cfg)
% Strapdown + 15-state error EKF with ZUPT (v=0) updates.

t = imuDataC.t(:);
gyro = imuDataC.gyro;
acc  = imuDataC.accel;
N = numel(t);

stationary = imu.zupt.detect_stationary(imuDataC, cfg);

% State (nominal)
p  = state0.p_W(:);
v  = state0.v_W(:);
q  = state0.q_WB(:);   % Body -> World quaternion
bg = state0.bg(:);
ba = state0.ba(:);

% Error-state covariance (15x15)
P = zeros(15);
P(1:3,1:3)   = 1e-4*eye(3);
P(4:6,4:6)   = 1e-3*eye(3);
P(7:9,7:9)   = 1e-3*eye(3);
P(10:12,10:12)= 1e-4*eye(3);
P(13:15,13:15)= 1e-3*eye(3);

gW = [0; -cfg.zupt.g0; 0];  % 너의 C-frame에서 -y가 중력이라는 Step2 결과 반영

% Noise params
sg  = cfg.zupt.sigma_g;
sa  = cfg.zupt.sigma_a;
sbg = cfg.zupt.sigma_bg;
sba = cfg.zupt.sigma_ba;

Rzupt = (cfg.zupt.sigma_zupt_v^2) * eye(3);

% Logs
traj.t = t;
traj.p = zeros(N,3);
traj.v = zeros(N,3);
traj.q = zeros(N,4);
traj.bg = zeros(N,3);
traj.ba = zeros(N,3);

dbg.stationary = stationary;
dbg.Nzupt = 0;

for k = 1:N
    if k==1
        dt = median(diff(t));
    else
        dt = t(k)-t(k-1);
    end
    dt = max(dt, 1e-4);

    % --- Nominal propagation
    w = gyro(k,:).'- bg;       % rad/s
    f = acc(k,:).'- ba;        % m/s^2 (specific force in body/C frame)

    % Integrate attitude
    dq = imu.utils.quat_from_omega(w, dt);
    q = imu.utils.quat_mul(q, dq);
    q = q / norm(q);

    Cwb = imu.utils.quat_to_dcm(q); % maps body(C-frame) -> world

    aW = Cwb * f + gW;

    v = v + aW * dt;
    p = p + v * dt;

    % --- Error-state EKF propagation (simple discrete)
    F = zeros(15);
    % δp_dot = δv
    F(1:3,4:6) = eye(3);
    % δv_dot ≈ -Cwb*[f]_x δθ - Cwb δba
    F(4:6,7:9)   = -Cwb * skew(f);
    F(4:6,13:15) = -Cwb;
    % δθ_dot ≈ -[w]_x δθ - δbg
    F(7:9,7:9)   = -skew(w);
    F(7:9,10:12) = -eye(3);
    % bias random walk
    % δbg_dot = noise, δba_dot = noise (handled in Q)

    Phi = eye(15) + F*dt;

    Q = zeros(15);
    Q(7:9,7:9)     = (sg^2)*dt*eye(3);
    Q(4:6,4:6)     = (sa^2)*dt*eye(3);
    Q(10:12,10:12) = (sbg^2)*dt*eye(3);
    Q(13:15,13:15) = (sba^2)*dt*eye(3);

    P = Phi*P*Phi.' + Q;

    % --- ZUPT update if stationary
    if cfg.zupt.enable && stationary(k)
        % Measurement: v = 0  => residual r = 0 - v
        r = -v;

        H = zeros(3,15);
        H(:,4:6) = eye(3);  % δv

        Szz = H*P*H.' + Rzupt;
        K = (P*H.') / Szz;

        dx = K * r;
        P = (eye(15) - K*H) * P;

        % Inject error into nominal
        dp  = dx(1:3);
        dv  = dx(4:6);
        dth = dx(7:9);
        dbgx= dx(10:12);
        dbax= dx(13:15);

        p  = p + dp;
        v  = v + dv;

        dq_corr = imu.utils.quat_from_small_angle(dth);
        q = imu.utils.quat_mul(q, dq_corr);
        q = q / norm(q);

        bg = bg + dbgx;
        ba = ba + dbax;

        dbg.Nzupt = dbg.Nzupt + 1;
    end

    % log
    traj.p(k,:)  = p.';
    traj.v(k,:)  = v.';
    traj.q(k,:)  = q.';
    traj.bg(k,:) = bg.';
    traj.ba(k,:) = ba.';
end
end

function S = skew(a)
S = [  0   -a(3)  a(2);
      a(3)  0    -a(1);
     -a(2) a(1)   0  ];
end
