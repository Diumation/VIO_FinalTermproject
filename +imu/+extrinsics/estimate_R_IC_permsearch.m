function [R_IC_best, dbg] = estimate_R_IC_permsearch(imuData, voLog, cfg)
% Estimate R_IC (IMU->Camera) by searching axis permutation/sign candidates
% Constraint: Camera yaw axis (C-y) aligns with IMU x-axis (I-x) up to sign.
%
% Scoring:
%  (A) yaw-rate correlation: omega_Cy vs VO yaw-rate
%  (B) gravity alignment: mean accel direction mapped close to C-y (down)

tI = imuData.t(:);
gI = imuData.gyro;    % Nx3
aI = imuData.accel;   % Nx3

tV = voLog.t(:);

% --- VO yaw-rate (use your existing VO dyaw)
dtV = diff(tV);
yawRateV = zeros(size(tV));
yawRateV(2:end) = voLog.dyaw(2:end) ./ max(1e-6, dtV);  % rad/s

% --- IMU gyro aligned to VO time (nearest)
idxI = nearest_index(tI, tV);
gI_atV = gI(idxI,:);  % length(tV) x 3

% --- Use mean accel direction as "gravity-ish" proxy (works even w/o stationary in many cases)
aMean = mean(aI, 1).';
aMean = aMean / max(1e-9, norm(aMean));

% --- Candidate generation
% Fix C-y = sY * I-x, with sY in {+1,-1}
% Remaining axes: C-x and C-z are ±I-y and ±I-z in some order, enforcing right-handedness det=+1.
cands = {};
for sY = [+1, -1]
    ey = sY * [1;0;0]; % C-y in IMU basis (column vector in IMU coords)

    for perm = 1:2
        if perm == 1
            baseX = [0;1;0]; baseZ = [0;0;1]; % C-x from I-y, C-z from I-z
        else
            baseX = [0;0;1]; baseZ = [0;1;0]; % swapped
        end

        for sX = [+1, -1]
            for sZ = [+1, -1]
                ex = sX * baseX;
                ez = sZ * baseZ;

                % Assemble R_CI (maps IMU vector to Camera vector): v_C = R_CI * v_I
                % Rows are camera axes expressed in IMU basis, equivalently columns of R_IC? We use row form:
                R = [ex.'; ey.'; ez.'];

                % Enforce orthonormal and right-handed
                if abs(det(R) - 1) > 1e-6
                    continue;
                end
                cands{end+1} = R; %#ok<AGROW>
            end
        end
    end
end

if isempty(cands)
    error("No valid axis-permutation candidates generated.");
end

% --- Score each candidate
scores = zeros(numel(cands),1);
detail = struct("yawCorr",[],"gravAlign",[]);

% Normalize VO yaw-rate segment: ignore near-zero to avoid noise domination
maskV = abs(yawRateV) > 0.05; % rad/s threshold
if nnz(maskV) < 20
    % if not enough excitation, relax
    maskV = true(size(yawRateV));
end

for i = 1:numel(cands)
    R_CI = cands{i};

    % (A) yaw-rate correlation: omega_Cy from IMU gyro
    omegaC = (R_CI * gI_atV.').';    % length(tV) x 3
    yawRateI = omegaC(:,2);

    c = corrcoef(yawRateI(maskV), yawRateV(maskV));
    if numel(c) >= 4
        yawCorr = c(1,2);
    else
        yawCorr = 0;
    end
    if ~isfinite(yawCorr), yawCorr = 0; end

    % (B) gravity alignment: mapped mean accel should point ~ +C-y (down)
    aC_mean = R_CI * aMean;
    gravAlign = aC_mean(2);  % dot with [0;1;0]
    % prefer positive alignment (down). If you use y-up, flip sign here.

    % Weighted score
    % yawCorr in [-1,1], gravAlign in [-1,1]
    scores(i) = 0.7*yawCorr + 0.3*gravAlign;

    detail(i).yawCorr = yawCorr; %#ok<AGROW>
    detail(i).gravAlign = gravAlign; %#ok<AGROW>
end

[~, ibest] = max(scores);
R_IC_best = cands{ibest};

dbg.candidates = cands;
dbg.scores = scores;
dbg.detail = detail;
dbg.bestIndex = ibest;

% Print summary
fprintf("[R_IC search] best score=%.3f | yawCorr=%.3f | gravAlign=%.3f\n", ...
    scores(ibest), detail(ibest).yawCorr, detail(ibest).gravAlign);
fprintf("[R_IC search] R_IC (IMU->Cam) = \n");
disp(R_IC_best);

end

function idx = nearest_index(tI, tV)
idx = zeros(size(tV));
for k = 1:numel(tV)
    [~, idx(k)] = min(abs(tI - tV(k)));
end
end
