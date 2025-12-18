function R_IC = estimate_R_IC(imuData, voLog, cfg)
% Estimate R_IC using rotation alignment (no stationary required)
% omega_C ≈ R_IC * omega_I

% --- 1) Time align (nearest neighbor)
t_imu = imuData.t;
t_vo  = voLog.t;

gyro_I = imuData.gyro;

% VO relative yaw -> angular velocity in camera frame
% assume small dt: omega_y ≈ dyaw / dt
dyaw = voLog.dyaw;          % rad
dt_vo = diff(t_vo);
omega_y = dyaw(2:end) ./ dt_vo;

% build camera-frame angular velocity vectors
omega_C = [zeros(numel(omega_y),1), omega_y(:), zeros(numel(omega_y),1)];

% match IMU gyro samples
omega_I = zeros(size(omega_C));
for k = 1:numel(omega_y)
    [~, idx] = min(abs(t_imu - t_vo(k+1)));
    omega_I(k,:) = gyro_I(idx,:);
end

% remove low-rotation samples (noise)
th = 0.05; % rad/s
mask = abs(omega_y) > th;
omega_I = omega_I(mask,:);
omega_C = omega_C(mask,:);

assert(size(omega_I,1) > 20, "Not enough rotational excitation to estimate R_IC");

% --- 2) Solve Wahba problem
H = omega_C.' * omega_I;
[U,~,V] = svd(H);
R_IC = U * diag([1 1 det(U*V.')]) * V.';
end
