function dq = quat_from_omega(w, dt)
% w: 3x1 rad/s, dt: s
theta = norm(w)*dt;
if theta < 1e-8
    dq = [1; 0; 0; 0];
    return;
end
axis = (w / norm(w));
half = 0.5*theta;
dq = [cos(half); axis*sin(half)];
end
