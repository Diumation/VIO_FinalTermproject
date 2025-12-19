function feat = detect_features(I, cfg)

switch upper(cfg.featureType)
    case "ORB"
        pts = detectORBFeatures(I);
        pts = selectStrongest(pts, cfg.numFeatures);
        [desc, pts] = extractFeatures(I, pts, "Method","ORB");

    case "SURF"
        pts = detectSURFFeatures(I, "MetricThreshold", 500);
        pts = selectStrongest(pts, min(cfg.numFeatures, pts.Count));
        [desc, pts] = extractFeatures(I, pts, "Method","SURF");

    case "BRISK"
        pts = detectBRISKFeatures(I);
        pts = selectStrongest(pts, cfg.numFeatures);
        [desc, pts] = extractFeatures(I, pts, "Method","BRISK");

    case "HARRIS"
        pts = detectHarrisFeatures(I);
        pts = selectStrongest(pts, cfg.numFeatures);
        [desc, pts] = extractFeatures(I, pts, "Method","FREAK"); % or "BRISK" if FREAK unavailable

    otherwise
        error("Unknown featureType: %s", cfg.featureType);
end

feat.points = pts;
feat.desc   = desc;
end
