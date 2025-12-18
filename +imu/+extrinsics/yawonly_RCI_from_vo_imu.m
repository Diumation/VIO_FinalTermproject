function [R_CI, dbg] = yawonly_RCI_from_vo_imu(tV, yawRateV, tI, gyroYawI)
% Estimate yaw-only IMU->Camera rotation constraint using VO yaw-rate and IMU gyro yaw axis.
% Assumptions:
%   - gyroYawI is the IMU RAW yaw axis signal (here gyro_x).
%   - Camera yaw axis is C-y (x-right, y-down, z-forward).
% Returns:
%   R_CI such that omega_C = R_CI * omega_I, with only yaw axis constraint enforced.

tV = tV(:); yawRateV = yawRateV(:);
tI = tI(:); gyroYawI = gyroYawI(:);

% Common grid on IMU time (VO is sparse)
tMin = max(min(tI), min(tV));
tMax = min(max(tI), max(tV));
maskI = (tI>=tMin) & (tI<=tMax);

tI2 = tI(maskI);
g2  = gyroYawI(maskI);

% Interpolate VO onto IMU grid (we will shift VO time by dt)
dtI = median(diff(tI2));
fs  = 1/max(1e-6, dtI);

% Normalize signals (remove DC)
g0 = g2 - mean(g2, "omitnan");

% Precompute VO on unshifted time (we'll shift via evaluation on (tI2 - dt))
% But simpler: do xcorr on uniform resampled grid.
tg = tI2;  % already uniform-ish after preprocess

% Candidate signs
signs = [+1, -1];

best.corr = -Inf;
best.dt   = 0;
best.sign = +1;

maxLagSec = 2.0;
maxLag = max(1, round(maxLagSec * fs));

for s = signs
    % VO sampled on IMU grid (initial, no shift)
    vo0 = interp1(tV, yawRateV, tg, 'linear', 'extrap');
    vo0 = vo0 - mean(vo0, "omitnan");

    % Focus on rotation-active samples (avoid noise domination)
    mask = (abs(vo0) > 0.05) | (abs(g0) > 0.05);
    x = vo0(mask);
    y = s * g0(mask);

    if numel(x) < 200
        % If too few, fall back without mask
        x = vo0; y = s*g0;
        x(~isfinite(x)) = 0; y(~isfinite(y)) = 0;
    end

    [c,lags] = xcorr(x, y, maxLag, 'coeff');
    [cmax, k] = max(c);
    lag = lags(k);

    dt = -lag / fs;  % shift VO time by +dt to align to IMU

    if cmax > best.corr
        best.corr = cmax;
        best.dt   = dt;
        best.sign = s;
    end
end

dbg.dt   = best.dt;
dbg.sign = best.sign;
dbg.corr = best.corr;

fprintf("[yawonly] best sign=%+d | dt=%.3f s | corr=%.3f\n", dbg.sign, dbg.dt, dbg.corr);

% Now build yaw-only R_CI:
% enforce omega_Cy ≈ (sign)*omega_Ix  => C-y axis = sign * I-x
% This sets row-2 of R_CI to [sign, 0, 0].
% For now choose C-x = I-y, C-z = I-z, then fix det.
Cx = [0;1;0];        % I-y
Cy = [dbg.sign;0;0]; % sign*I-x
Cz = [0;0;1];        % I-z

R_CI = [Cx.'; Cy.'; Cz.'];

% Ensure det +1 (proper rotation). If det=-1, flip Cx.
if det(R_CI) < 0
    Cx = -Cx;
    R_CI = [Cx.'; Cy.'; Cz.'];
end

end
