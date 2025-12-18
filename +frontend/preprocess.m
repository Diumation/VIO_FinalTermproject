function imu = preprocess(imuRaw, cfg)
% Optional: resample to uniform grid; optional lowpass.

t = imuRaw.t(:);
a = imuRaw.accel;
w = imuRaw.gyro;

% estimate fs
dt = diff(t);
fs_est = 1/median(dt(dt>0));

imu = imuRaw;
imu.meta.fs_est = fs_est;

if cfg.preproc.enableResample
    fs = cfg.preproc.fsTarget;
    t2 = (t(1):1/fs:t(end)).';
    a2 = interp1(t, a, t2, "linear", "extrap");
    w2 = interp1(t, w, t2, "linear", "extrap");
    t = t2; a = a2; w = w2;
    imu.meta.fs = fs;
else
    imu.meta.fs = fs_est;
end

% optional lowpass (simple 1st-order IIR via smoothdata if desired)
if cfg.preproc.lowpassHz > 0
    % light smoothing without toolbox dependencies
    win = max(3, round(imu.meta.fs / cfg.preproc.lowpassHz));
    a = movmean(a, win, 1);
    w = movmean(w, win, 1);
end

imu.t = t;
imu.accel = a;
imu.gyro  = w;
end
