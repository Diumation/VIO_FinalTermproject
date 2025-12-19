function rel = estimate_motion_2view_points(p1, p2, cfg)
K = cfg.K;

% Estimate Essential matrix
try
    [E, inlierIdx] = estimateEssentialMatrix(p1, p2, K, ...
        "Confidence", cfg.confidence, ...
        "MaxNumTrials", 3000);
catch
    rel.isValid = false; rel.numInliers = 0; rel.T_C1C2 = eye(4);
    return;
end

p1i = p1(inlierIdx,:);
p2i = p2(inlierIdx,:);
numIn = size(p1i,1);

if numIn < cfg.minInliers
    rel.isValid = false; rel.numInliers = numIn; rel.T_C1C2 = eye(4);
    return;
end

[orient, loc, validFraction] = relativeCameraPose(E, K, p1i, p2i);

if validFraction < 0.6
    rel.isValid = false; rel.numInliers = numIn; rel.T_C1C2 = eye(4);
    rel.validFraction = validFraction;
    return;
end

T = eye(4);
T(1:3,1:3) = orient;
T(1:3,4)   = loc(:);

rel.isValid = true;
rel.E = E;
rel.inlierIdx = inlierIdx;
rel.numInliers = numIn;
rel.validFraction = validFraction;
rel.T_C1C2 = T;
end
