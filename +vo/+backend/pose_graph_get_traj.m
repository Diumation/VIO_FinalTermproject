function traj = pose_graph_get_traj(state)
N = numel(state.frames);

t = zeros(N,1);
p = zeros(N,3);

for k = 1:N
    t(k) = state.frames(k).t;

    T = state.frames(k).T_WC;

    % 현재 파이프라인에서 T_WC의 translation을 "camera location"으로 사용
    % (단안이라 scale-free, SE3 엄밀성은 후속 단계에서 정리)
    p(k,:) = T(1:3,4).';
end

traj.t = t;
traj.pWvo = p;

% ---- Frame definition metadata
traj.meta.Wvo_def = state.cfg.frame.Wvo_def;

% ---- Auto detect up-axis: smallest variance axis
v = var(p, 0, 1);
[~, upIdx] = min(v);
groundAxes = setdiff(1:3, upIdx);

traj.meta.upAxis = upIdx;          % 1=X,2=Y,3=Z
traj.meta.groundAxes = groundAxes; % indices of the ground plane axes

% Quality logs (optional)
traj.meta.numFrames = N;
end
