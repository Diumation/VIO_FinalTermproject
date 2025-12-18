function stationary = detect_stationary(imu, cfg)
% Stationary if gyro magnitude small and accel magnitude close to g.

w = imu.gyro;
a = imu.accel;

wmag = sqrt(sum(w.^2,2));
amag = sqrt(sum(a.^2,2));

cond = (wmag < cfg.stationary.gyroTh) & (abs(amag - cfg.g) < cfg.stationary.accTh);

% enforce minimum duration
fs = imu.meta.fs;
minN = max(1, round(cfg.stationary.minDur * fs));

stationary = false(size(cond));
if minN == 1
    stationary = cond;
    return;
end

% run-length filter: keep only segments of length >= minN
d = diff([false; cond; false]);
starts = find(d==1);
ends   = find(d==-1)-1;

for i = 1:numel(starts)
    if ends(i)-starts(i)+1 >= minN
        stationary(starts(i):ends(i)) = true;
    end
end
end
