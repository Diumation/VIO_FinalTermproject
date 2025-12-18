function stationary = detect_stationary(imuDataC, cfg)
% stationary: Nx1 logical
% Criteria:
%   ||gyro|| < gyroThr  AND  | ||acc|| - g0 | < accThr
t = imuDataC.t(:);
gyro = imuDataC.gyro;
acc  = imuDataC.accel;

g0 = cfg.zupt.g0;
wNorm = vecnorm(gyro,2,2);
aNorm = vecnorm(acc,2,2);

raw = (wNorm < cfg.zupt.gyroThr) & (abs(aNorm - g0) < cfg.zupt.accThr);

% enforce minimum duration
dt = median(diff(t));
minSamples = max(1, round(cfg.zupt.minDur / max(1e-6, dt)));

stationary = false(size(raw));
i = 1;
N = numel(raw);
while i <= N
    if ~raw(i), i = i + 1; continue; end
    j = i;
    while j <= N && raw(j), j = j + 1; end
    if (j - i) >= minSamples
        stationary(i:j-1) = true;
    end
    i = j;
end
end
