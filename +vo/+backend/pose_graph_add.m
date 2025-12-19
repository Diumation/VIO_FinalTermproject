function state = pose_graph_add(state, id, t, I, feat, T_WC, matches, rel)
if nargin < 7, matches = []; end
if nargin < 8, rel = []; end

k = numel(state.frames) + 1;
state.frames(k).id = id;
state.frames(k).t  = t;
state.frames(k).I  = I;
state.frames(k).feat = feat;
state.frames(k).T_WC = T_WC;
state.frames(k).matches = matches;
state.frames(k).rel = rel;
end
