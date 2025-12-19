function [klt, tracks] = klt_track(klt, Iprev, Icur, cfg)

[p2, valid] = step(klt.tracker, Icur);
p1 = klt.points(valid,:);
p2 = p2(valid,:);

% Optional: enforce spatial distribution (avoid clustering)
tracks.p1 = p1;
tracks.p2 = p2;
tracks.numTracked = size(p1,1);

% Update internal state
klt.points = p2;

% Update tracker points (remove lost)
setPoints(klt.tracker, p2);

end
