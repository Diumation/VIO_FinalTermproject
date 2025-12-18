function state = init_state(init, cfg)
state.t   = init.t0;
state.q_WB = init.q_WB0;      % World <- Body
state.v_W  = zeros(3,1);
state.p_W  = zeros(3,1);

state.bg = init.bg0;
state.ba = init.ba0;

state.g_W = init.g_W;
end
