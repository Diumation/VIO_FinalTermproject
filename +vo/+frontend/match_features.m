function matches = match_features(prev, cur, cfg)
idxPairs = matchFeatures(prev.feat.desc, cur.feat.desc, ...
    "Unique", cfg.uniqueMatch, ...
    "MaxRatio", cfg.maxRatio);

m1 = prev.feat.points(idxPairs(:,1));
m2 = cur.feat.points(idxPairs(:,2));

matches.idxPairs = idxPairs;
matches.m1 = m1;  % matched points in prev
matches.m2 = m2;  % matched points in cur
matches.numMatches = size(idxPairs,1);
matches.numInliers = 0; % filled later
end
