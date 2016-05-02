function [est_C_b_e_new,est_v_eb_e_new,est_r_eb_e_new,est_IMU_bias_new,...
            P_matrix_new] = LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,...
            est_C_b_e_old,est_v_eb_e_old,est_r_eb_e_old,est_IMU_bias_old,...
            P_matrix_old,meas_f_ib_b,est_L_b_old,LC_KF_config)
%LC_KF_Epoch - Implements one cycle of the loosely coupled INS/GNSS
% Kalman filter plus closed-loop correction of all inertial states
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   GNSS_r_eb_e           GNSS estimated ECEF user position (m)
%   GNSS_v_eb_e           GNSS estimated ECEF user velocity (m/s)
%   tor_s                 propagation interval (s)
%   est_C_b_e_old         prior estimated body to ECEF coordinate
%                         transformation matrix
%   est_v_eb_e_old        prior estimated ECEF user velocity (m/s)
%   est_r_eb_e_old        prior estimated ECEF user position (m)
%   est_IMU_bias_old      prior estimated IMU biases (body axes)
%   P_matrix_old          previous Kalman filter error covariance matrix
%   meas_f_ib_b           measured specific force
%   est_L_b_old           previous latitude solution
%   LC_KF_config
%     .gyro_noise_PSD     Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD    Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD     Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD      Gyro bias random walk PSD (rad^2 s^-3)
%     .pos_meas_SD            Position measurement noise SD per axis (m)
%     .vel_meas_SD            Velocity measurement noise SD per axis (m/s)
%
% Outputs:
%   est_C_b_e_new     updated estimated body to ECEF coordinate 
%                      transformation matrix
%   est_v_eb_e_new    updated estimated ECEF user velocity (m/s)
%   est_r_eb_e_new    updated estimated ECEF user position (m)
%   est_IMU_bias_new  updated estimated IMU biases
%     Rows 1-3          estimated accelerometer biases (m/s^2) 
%     Rows 4-6          estimated gyro biases (rad/s)
%   P_matrix_new      updated Kalman filter error covariance matrix

 
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Begins

% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);
       
% SYSTEM PROPAGATION PHASE

% 1. Determine transition matrix using (14.50) (first-order approx)
Phi_matrix = eye(15);
Phi_matrix(1:3,1:3) = Phi_matrix(1:3,1:3) - Omega_ie * tor_s;
Phi_matrix(1:3,13:15) = est_C_b_e_old * tor_s;
Phi_matrix(4:6,1:3) = -tor_s * Skew_symmetric(est_C_b_e_old * meas_f_ib_b);
Phi_matrix(4:6,4:6) = Phi_matrix(4:6,4:6) - 2 * Omega_ie * tor_s;
geocentric_radius = R_0 / sqrt(1 - (e * sin(est_L_b_old))^2) *...
    sqrt(cos(est_L_b_old)^2 + (1 - e^2)^2 * sin(est_L_b_old)^2); % from (2.137)
Phi_matrix(4:6,7:9) = -tor_s * 2 * Gravity_ECEF(est_r_eb_e_old) /...
    geocentric_radius * est_r_eb_e_old' / sqrt (est_r_eb_e_old' *...
    est_r_eb_e_old);
Phi_matrix(4:6,10:12) = est_C_b_e_old * tor_s;
Phi_matrix(7:9,4:6) = eye(3) * tor_s;

% 2. Determine approximate system noise covariance matrix using (14.82)
Q_prime_matrix = zeros(15);
Q_prime_matrix(1:3,1:3) = eye(3) * LC_KF_config.gyro_noise_PSD * tor_s;
Q_prime_matrix(4:6,4:6) = eye(3) * LC_KF_config.accel_noise_PSD * tor_s;
Q_prime_matrix(10:12,10:12) = eye(3) * LC_KF_config.accel_bias_PSD * tor_s;
Q_prime_matrix(13:15,13:15) = eye(3) * LC_KF_config.gyro_bias_PSD * tor_s;

% 3. Propagate state estimates using (3.14) noting that all states are zero 
% due to closed-loop correction.
x_est_propagated(1:15,1) = 0;

% 4. Propagate state estimation error covariance matrix using (3.46)
P_matrix_propagated = Phi_matrix * (P_matrix_old + 0.5 * Q_prime_matrix) *...
    Phi_matrix' + 0.5 * Q_prime_matrix;

% MEASUREMENT UPDATE PHASE
       
% 5. Set-up measurement matrix using (14.115)
H_matrix = zeros(6,15);
H_matrix(1:3,7:9) = -eye(3);
H_matrix(4:6,4:6) = -eye(3);

% 6. Set-up measurement noise covariance matrix assuming all components of
% GNSS position and velocity are independent and have equal variance.
R_matrix(1:3,1:3) = eye(3) * LC_KF_config.pos_meas_SD^2;
R_matrix(1:3,4:6) = zeros(3);
R_matrix(4:6,1:3) = zeros(3);
R_matrix(4:6,4:6) = eye(3) * LC_KF_config.vel_meas_SD^2;

% 7. Calculate Kalman gain using (3.21)
K_matrix = P_matrix_propagated * H_matrix' * inv(H_matrix *...
    P_matrix_propagated * H_matrix' + R_matrix);

% 8. Formulate measurement innovations using (14.102), noting that zero
% lever arm is assumed here
delta_z(1:3,1) = GNSS_r_eb_e -est_r_eb_e_old;
delta_z(4:6,1) = GNSS_v_eb_e -est_v_eb_e_old;

% 9. Update state estimates using (3.24)
x_est_new = x_est_propagated + K_matrix * delta_z;

% 10. Update state estimation error covariance matrix using (3.25)
P_matrix_new = (eye(15) - K_matrix * H_matrix) * P_matrix_propagated;

% CLOSED-LOOP CORRECTION

% Correct attitude, velocity, and position using (14.7-9)
est_C_b_e_new = (eye(3) - Skew_symmetric(x_est_new(1:3))) * est_C_b_e_old;
est_v_eb_e_new = est_v_eb_e_old - x_est_new(4:6);
est_r_eb_e_new = est_r_eb_e_old - x_est_new(7:9);

% Update IMU bias estimates
est_IMU_bias_new = est_IMU_bias_old + x_est_new(10:15);

% Ends