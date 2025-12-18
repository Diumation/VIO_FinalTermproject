function imuRaw = load_imu_csv(csvPath, cfg)
% Load IMU CSV with columns:
% timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z

T = readtable(csvPath);

required = ["timestamp","a_x","a_y","a_z","alpha_x","alpha_y","alpha_z"];
for c = required
    if ~any(strcmp(T.Properties.VariableNames, c))
        error("IMU CSV missing required column: %s", c);
    end
end

t = double(T.timestamp);
accel = [double(T.a_x), double(T.a_y), double(T.a_z)];
gyro  = [double(T.alpha_x), double(T.alpha_y), double(T.alpha_z)];

% gyro unit conversion
if isfield(cfg, "gyroUnit") && strcmpi(cfg.gyroUnit, "deg/s")
    gyro = gyro * (pi/180);
end

imuRaw.t = t(:);
imuRaw.accel = accel;
imuRaw.gyro  = gyro;
imuRaw.meta.source = "csv";
imuRaw.meta.path = string(csvPath);
end
