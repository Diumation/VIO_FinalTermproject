function traj = integrate(imuData, state0, cfg)
% IMU dead-reckoning integration loop
% imuData: struct with t, accel, gyro
% state0 : initial state

t = imuData.t(:);
a = imuData.accel;
w = imuData.gyro;

N = numel(t);
s = state0;

traj = [];
traj = imu.backend.log_add(traj, s, 1);

for k = 2:N
    dt = t(k) - t(k-1);

    meas.accel = a(k,:).';
    meas.gyro  = w(k,:).';

    s = imu.prop.strapdown_step(s, meas, dt, cfg);

    traj = imu.backend.log_add(traj, s, k);
end
end
