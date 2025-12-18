function R = rot_from_two_vectors(u, v)
u = u / norm(u); v = v / norm(v);
axis = cross(u,v);
if norm(axis) < 1e-6
    R = eye(3); return;
end
axis = axis / norm(axis);
ang = acos(dot(u,v));
R = axang2rotm([axis.' ang]);
end
