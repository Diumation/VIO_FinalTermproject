function imuRaw = sanity_checks(imuRaw, cfg)
t = imuRaw.t;

if any(~isfinite(t))
    error("IMU timestamp contains non-finite values.");
end

% remove NaNs in measurements
mask = all(isfinite(imuRaw.accel),2) & all(isfinite(imuRaw.gyro),2) & isfinite(t);
imuRaw.t     = imuRaw.t(mask);
imuRaw.accel = imuRaw.accel(mask,:);
imuRaw.gyro  = imuRaw.gyro(mask,:);

% enforce monotonic time (optional)
if cfg.time.removeNonMonotonic
    dt = diff(imuRaw.t);
    good = [true; dt > 0];
    imuRaw.t     = imuRaw.t(good);
    imuRaw.accel = imuRaw.accel(good,:);
    imuRaw.gyro  = imuRaw.gyro(good,:);
end

% start at zero (recommended)
if cfg.time.makeStartAtZero
    imuRaw.t = imuRaw.t - imuRaw.t(1);
end
end
