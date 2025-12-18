function qn = quat_normalize(q)
n = imu.models.quat_norm(q);
qn = q / max(1e-12, n);
end
