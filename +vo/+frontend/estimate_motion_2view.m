function rel = estimate_motion_2view(prev, cur, matches, cfg)
K = cfg.K;

p1 = matches.m1.Location;   % Nx2
p2 = matches.m2.Location;

% 1) Essential matrix with RANSAC (use intrinsics directly)
try
    [E, inlierIdx] = estimateEssentialMatrix(p1, p2, K, ...
        "Confidence", cfg.confidence, ...
        "MaxNumTrials", 3000);
catch ME
    warning("estimateEssentialMatrix failed: %s", ME.message);
    rel.isValid = false;
    rel.T_C1C2 = eye(4);
    rel.numInliers = 0;
    return;
end

p1i = p1(inlierIdx,:);
p2i = p2(inlierIdx,:);
numIn = size(p1i,1);

if numIn < cfg.minInliers
    rel.isValid = false;
    rel.T_C1C2 = eye(4);
    rel.E = E;
    rel.inlierIdx = inlierIdx;
    rel.numInliers = numIn;
    return;
end

% 2) Recover relative pose
[orient, loc, validFraction] = relativeCameraPose(E, K, p1i, p2i);

if validFraction < cfg.cheiralityMinPositiveDepth
    rel.isValid = false;
    rel.T_C1C2 = eye(4);
    rel.E = E;
    rel.inlierIdx = inlierIdx;
    rel.numInliers = numIn;
    rel.validFraction = validFraction;
    return;
end

R = orient;
t = loc(:); % translation direction (scale unknown)

T = eye(4);
T(1:3,1:3) = R;
T(1:3,4)   = t;

rel.isValid = true;
rel.E = E;
rel.inlierIdx = inlierIdx;
rel.numInliers = numIn;
rel.validFraction = validFraction;
rel.T_C1C2 = T;
rel.R = R;
rel.t = t;
end
