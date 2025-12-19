function klt = klt_init(I0, cfg)
pts = detectMinEigenFeatures(I0, "MinQuality", 0.01);
pts = selectStrongest(pts, cfg.klt.detectPerRefresh);

tracker = vision.PointTracker( ...
    "MaxBidirectionalError", cfg.klt.maxBidirectionalError, ...
    "NumPyramidLevels", cfg.klt.numPyramidLevels, ...
    "BlockSize", cfg.klt.blockSize);

initialize(tracker, pts.Location, I0);

klt.tracker = tracker;
klt.points  = pts.Location;     % Nx2 current points
klt.feat0.points = pts;         % keep for logging
end
