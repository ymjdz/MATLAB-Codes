function [x_est_new,P_matrix_new] = GNSS_KF_Epoch(GNSS_measurements,...
            no_meas,tor_s,x_est_old,P_matrix_old,GNSS_KF_config)
%GNSS_KF_Epoch - Implements one cycle of the GNSS extended Kalman filter
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   GNSS_measurements     GNSS measurement data:
%     Column 1              Pseudo-range measurements (m)
%     Column 2              Pseudo-range rate measurements (m/s)
%     Columns 3-5           Satellite ECEF position (m)
%     Columns 6-8           Satellite ECEF velocity (m/s)
%   no_meas               Number of satellites for which measurements are
%                         supplied
%   tor_s                 propagation interval (s)
%   x_est_old             previous Kalman filter state estimates
%   P_matrix_old          previous Kalman filter error covariance matrix
%   GNSS_KF_config
%     .accel_PSD              Acceleration PSD per axis (m^2/s^3)
%     .clock_freq_PSD         Receiver clock frequency-drift PSD (m^2/s^3)
%     .clock_phase_PSD        Receiver clock phase-drift PSD (m^2/s)
%     .pseudo_range_SD        Pseudo-range measurement noise SD (m)
%     .range_rate_SD          Pseudo-range rate measurement noise SD (m/s)
%
% Outputs:
%   x_est_new             updated Kalman filter state estimates
%     Rows 1-3            estimated ECEF user position (m)
%     Rows 4-6            estimated ECEF user velocity (m/s)
%     Row 7               estimated receiver clock offset (m) 
%     Row 8               estimated receiver clock drift (m/s)
%   P_matrix_new          updated Kalman filter error covariance matrix

 
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s

% Begins

% SYSTEM PROPAGATION PHASE

% 1. Determine transition matrix using (9.147) and (9.150)
Phi_matrix = eye(8);
Phi_matrix(1,4) = tor_s;
Phi_matrix(2,5) = tor_s;
Phi_matrix(3,6) = tor_s;
Phi_matrix(7,8) = tor_s;

% 2. Determine system noise covariance matrix using (9.152)
Q_matrix = zeros(8);
Q_matrix(1:3,1:3) = eye(3) * GNSS_KF_config.accel_PSD * tor_s^3 / 3;
Q_matrix(1:3,4:6) = eye(3) * GNSS_KF_config.accel_PSD * tor_s^2 / 2;
Q_matrix(4:6,1:3) = eye(3) * GNSS_KF_config.accel_PSD * tor_s^2 / 2;
Q_matrix(4:6,4:6) = eye(3) * GNSS_KF_config.accel_PSD * tor_s;
Q_matrix(7,7) = (GNSS_KF_config.clock_freq_PSD * tor_s^3 / 3) +...
    GNSS_KF_config.clock_phase_PSD * tor_s;
Q_matrix(7,8) = GNSS_KF_config.clock_freq_PSD * tor_s^2 / 2;
Q_matrix(8,7) = GNSS_KF_config.clock_freq_PSD * tor_s^2 / 2;
Q_matrix(8,8) = GNSS_KF_config.clock_freq_PSD * tor_s;

% 3. Propagate state estimates using (3.14)
x_est_propagated = Phi_matrix * x_est_old;

% 4. Propagate state estimation error covariance matrix using (3.15)
P_matrix_propagated = Phi_matrix * P_matrix_old * Phi_matrix' + Q_matrix;

% MEASUREMENT UPDATE PHASE

% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);
       
u_as_e_T = zeros(no_meas,3);
pred_meas = zeros(no_meas,2);

% Loop measurements
for j = 1:no_meas

    % Predict approx range 
    delta_r = GNSS_measurements(j,3:5)' - x_est_propagated(1:3);
    approx_range = sqrt(delta_r' * delta_r);

    % Calculate frame rotation during signal transit time using (8.36)
    C_e_I = [1, omega_ie * approx_range / c, 0;...
             -omega_ie * approx_range / c, 1, 0;...
             0, 0, 1];

    % Predict pseudo-range using (9.165)
    delta_r = C_e_I *  GNSS_measurements(j,3:5)' - x_est_propagated(1:3);
    range = sqrt(delta_r' * delta_r);
    pred_meas(j,1) = range + x_est_propagated(7);
        
    % Predict line of sight
    u_as_e_T(j,1:3) = delta_r' / range;
        
    % Predict pseudo-range rate using (9.165)
    range_rate = u_as_e_T(j,1:3) * (C_e_I * (GNSS_measurements(j,6:8)' +...
        Omega_ie * GNSS_measurements(j,3:5)') - (x_est_propagated(4:6) +...
        Omega_ie * x_est_propagated(1:3)));        
    pred_meas(j,2) = range_rate + x_est_propagated(8);

end % for j
        
% 5. Set-up measurement matrix using (9.163)
H_matrix(1:no_meas,1:3) = -u_as_e_T(1:no_meas,1:3);
H_matrix(1:no_meas,4:6) = zeros(no_meas,3);
H_matrix(1:no_meas,7) = ones(no_meas,1);
H_matrix(1:no_meas,8) = zeros(no_meas,1);
H_matrix((no_meas + 1):(2 * no_meas),1:3) = zeros(no_meas,3);
H_matrix((no_meas + 1):(2 * no_meas),4:6) = -u_as_e_T(1:no_meas,1:3);
H_matrix((no_meas + 1):(2 * no_meas),7) = zeros(no_meas,1);
H_matrix((no_meas + 1):(2 * no_meas),8) = ones(no_meas,1);

% 6. Set-up measurement noise covariance matrix assuming all measurements
% are independent and have equal variance for a given measurement type
R_matrix(1:no_meas,1:no_meas) = eye(no_meas) *...
    GNSS_KF_config.pseudo_range_SD^2;
R_matrix(1:no_meas,(no_meas + 1):(2 * no_meas)) =...
    zeros(no_meas);
R_matrix((no_meas + 1):(2 * no_meas),1:no_meas) =...
    zeros(no_meas);
R_matrix((no_meas + 1):(2 * no_meas),(no_meas + 1):(2 * no_meas)) =...
    eye(no_meas) * GNSS_KF_config.range_rate_SD^2;

% 7. Calculate Kalman gain using (3.21)
K_matrix = P_matrix_propagated * H_matrix' * inv(H_matrix *...
    P_matrix_propagated * H_matrix' + R_matrix);

% 8. Formulate measurement innovations using (3.88)
delta_z(1:no_meas,1) = GNSS_measurements(1:no_meas,1) -...
    pred_meas(1:no_meas,1);
delta_z((no_meas + 1):(2 * no_meas),1) = GNSS_measurements(1:no_meas,2) -...
    pred_meas(1:no_meas,2);

% 9. Update state estimates using (3.24)
x_est_new = x_est_propagated + K_matrix * delta_z;

% 10. Update state estimation error covariance matrix using (3.25)
P_matrix_new = (eye(8) - K_matrix * H_matrix) * P_matrix_propagated;

% Ends