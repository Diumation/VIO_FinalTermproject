function state = pose_graph_init(cfg)
state.cfg = cfg;

state.frames = struct( ...
    "id", {}, "t", {}, "I", {}, "feat", {}, ...
    "T_WC", {}, "matches", {}, "rel", {} );

end
