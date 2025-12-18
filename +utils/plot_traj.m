function plot_traj(traj, cfg)
figure;
plot(traj.p_W(:,1), traj.p_W(:,3), "r-"); grid on;
xlabel("X"); ylabel("Z");
title("IMU DR trajectory (X-Z, arbitrary scale/drift expected)");

figure;
plot(traj.t, traj.v_W); grid on;
xlabel("time (s)"); ylabel("v_W (m/s)");
legend("v_x","v_y","v_z");
title("Velocity in World");
end
