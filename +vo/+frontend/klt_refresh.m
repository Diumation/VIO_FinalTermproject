function klt = klt_refresh(I, cfg)
% release old tracker if exists
try
    release(cfg.klt.tracker);
catch
end

pts = detectMinEigenFeatures(I, "MinQuality", 0.01);
pts = selectStrongest(pts, cfg.klt.detectPerRefresh);

tracker = vision.PointTracker( ...
    "MaxBidirectionalError", cfg.klt.maxBidirectionalError, ...
    "NumPyramidLevels", cfg.klt.numPyramidLevels, ...
    "BlockSize", cfg.klt.blockSize);

initialize(tracker, pts.Location, I);

klt.tracker = tracker;
klt.points  = pts.Location;
klt.feat0.points = pts;
end
