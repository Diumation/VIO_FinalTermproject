function imuData = apply_R_IC(imuData, R_IC)
imuData.accel = (R_IC * imuData.accel.').';
imuData.gyro  = (R_IC * imuData.gyro.').';
end
