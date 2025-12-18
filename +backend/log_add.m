function log = log_add(log, s, k)
% Append state to trajectory log
if isempty(log)
    log.t   = s.t;
    log.p_W = s.p_W.';
    log.v_W = s.v_W.';
    log.q_WB = s.q_WB.';
    log.bg = s.bg.';
    log.ba = s.ba.';
    log.k = k;
    return;
end

log.t(end+1,1)   = s.t;
log.p_W(end+1,:) = s.p_W.';
log.v_W(end+1,:) = s.v_W.';
log.q_WB(end+1,:) = s.q_WB.';
log.bg(end+1,:) = s.bg.';
log.ba(end+1,:) = s.ba.';
log.k(end+1,1) = k;
end
