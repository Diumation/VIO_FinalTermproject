function state = pose_graph_add_klt(state, id, t, I, T_WC, tracks, rel)
k = numel(state.frames) + 1;
state.frames(k).id = id;
state.frames(k).t  = t;
state.frames(k).I  = I;
state.frames(k).T_WC = T_WC;

state.frames(k).tracks = tracks;   % p1,p2,numTracked
state.frames(k).rel = rel;         % E, inliers, T_C1C2, validFraction
end
