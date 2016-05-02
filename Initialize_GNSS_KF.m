function [x_est,P_matrix] = Initialize_GNSS_KF(est_r_ea_e,est_v_ea_e,...
    est_clock,GNSS_KF_config)
%Initialize_GNSS_KF - Initializes the GNSS EKF state estimates and error
%covariance matrix
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   est_r_ea_e            estimated ECEF user position (m)
%   est_v_ea_e            estimated ECEF user velocity (m/s)
%   est_clock             estimated receiver clock offset (m) and drift (m/s)
%   GNSS_KF_config
%     .init_pos_unc           Initial position uncertainty per axis (m)
%     .init_vel_unc           Initial velocity uncertainty per axis (m/s)
%     .init_clock_offset_unc  Initial clock offset uncertainty per axis (m)
%     .init_clock_drift_unc   Initial clock drift uncertainty per axis (m/s)
%
% Outputs:
%   x_est                 Kalman filter estimates:
%     Rows 1-3            estimated ECEF user position (m)
%     Rows 4-6            estimated ECEF user velocity (m/s)
%     Row 7               estimated receiver clock offset (m) 
%     Row 8               estimated receiver clock drift (m/s)
%   P_matrix              state estimation error covariance matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Initialize state estimates
x_est(1:3,1) = est_r_ea_e;
x_est(4:6,1) = est_v_ea_e;
x_est(7:8,1) = est_clock';

% Initialize error covariance matrix
P_matrix =  zeros(8);
P_matrix(1,1) = GNSS_KF_config.init_pos_unc^2;
P_matrix(2,2) = GNSS_KF_config.init_pos_unc^2;
P_matrix(3,3) = GNSS_KF_config.init_pos_unc^2;
P_matrix(4,4) = GNSS_KF_config.init_vel_unc^2;
P_matrix(5,5) = GNSS_KF_config.init_vel_unc^2;
P_matrix(6,6) = GNSS_KF_config.init_vel_unc^2;
P_matrix(7,7) = GNSS_KF_config.init_clock_offset_unc^2;
P_matrix(8,8) = GNSS_KF_config.init_clock_drift_unc^2;

% Ends