%INS_GNSS_Demo_12
%SCRIPT Tightly coupled INS/GNSS demo:
%   Profile_3 (418s aircraft)
%   Aviation-grade IMU
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% Created 27/5/12 by Paul Groves

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;

% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_3.csv';
% Output motion profile and error filenames
output_profile_name = 'INS_GNSS_Demo_12_Profile.csv';
output_errors_name = 'INS_GNSS_Demo_12_Errors.csv';

% Attitude initialization error (deg, converted to rad; @N,E,D)
initialization_errors.delta_eul_nb_n = [-0.01;0.008;0.01]*deg_to_rad; % rad

% Accelerometer biases (micro-g, converted to m/s^2; body axes)
IMU_errors.b_a = [30;-45;26] * micro_g_to_meters_per_second_squared;
% Gyro biases (deg/hour, converted to rad/sec; body axes)
IMU_errors.b_g = [-0.0009;0.0013;-0.0008] * deg_to_rad / 3600;
% Accelerometer scale factor and cross coupling errors (ppm, converted to
% unitless; body axes)
IMU_errors.M_a = [100, -120,  80;...
                  -60, -120, 100;...
                 -100,   40,  90] * 1E-6;
% Gyro scale factor and cross coupling errors (ppm, converted to unitless;
% body axes)
IMU_errors.M_g = [8, -120, 100;...
                  0,   -6, -60;...
                  0,    0,  -7] * 1E-6;             
% Gyro g-dependent biases (deg/hour/g, converted to rad-sec/m; body axes)
IMU_errors.G_g = [0, 0, 0;...
                  0, 0, 0;...
                  0, 0, 0] * deg_to_rad / (3600 * 9.80665);             
% Accelerometer noise root PSD (micro-g per root Hz, converted to m s^-1.5)                
IMU_errors.accel_noise_root_PSD = 20 *...
    micro_g_to_meters_per_second_squared;
% Gyro noise root PSD (deg per root hour, converted to rad s^-0.5)                
IMU_errors.gyro_noise_root_PSD = 0.002 * deg_to_rad / 60;
% Accelerometer quantization level (m/s^2)
IMU_errors.accel_quant_level = 5E-5;
% Gyro quantization level (rad/s)
IMU_errors.gyro_quant_level = 1E-6;

% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.5;

% Initial estimated position (m; ECEF)
GNSS_config.init_est_r_ea_e = [0;0;0];

% Number of satellites in constellation
GNSS_config.no_sat = 30;
% Orbital radius of satellites (m)
GNSS_config.r_os = 2.656175E7;
% Inclination angle of satellites (deg)
GNSS_config.inclination = 55;
% Longitude offset of constellation (deg)
GNSS_config.const_delta_lambda = 0;
% Timing offset of constellation (s)
GNSS_config.const_delta_t = 0;

% Mask angle (deg)
GNSS_config.mask_angle = 10;
% Signal in space error SD (m) *Give residual where corrections are applied
GNSS_config.SIS_err_SD = 1;
% Zenith ionosphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_iono_err_SD = 2;
% Zenith troposphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_trop_err_SD = 0.2;
% Code tracking error SD (m) *Can extend to account for multipath
GNSS_config.code_track_err_SD = 1;
% Range rate tracking error SD (m/s) *Can extend to account for multipath
GNSS_config.rate_track_err_SD = 0.02;
% Receiver clock offset at time=0 (m);
GNSS_config.rx_clock_offset = 10000;
% Receiver clock drift at time=0 (m/s);
GNSS_config.rx_clock_drift = 100;

% Initial attitude uncertainty per axis (deg, converted to rad)
TC_KF_config.init_att_unc = degtorad(0.01);
% Initial velocity uncertainty per axis (m/s)
TC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
TC_KF_config.init_pos_unc = 10;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted
% to m/s^2)
TC_KF_config.init_b_a_unc = 30 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
TC_KF_config.init_b_g_unc = 0.001 * deg_to_rad / 3600;
% Initial clock offset uncertainty per axis (m)
TC_KF_config.init_clock_offset_unc = 10;
% Initial clock drift uncertainty per axis (m/s)
TC_KF_config.init_clock_drift_unc = 0.1;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
TC_KF_config.gyro_noise_PSD = (0.004 * deg_to_rad / 60)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
TC_KF_config.accel_noise_PSD = (40 *...
    micro_g_to_meters_per_second_squared)^2;
% Accelerometer bias random walk PSD (m^2 s^-5)
TC_KF_config.accel_bias_PSD = 3.0E-9;
% Gyro bias random walk PSD (rad^2 s^-3)
TC_KF_config.gyro_bias_PSD = 2.0E-16;
% Receiver clock frequency-drift PSD (m^2/s^3)
TC_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
TC_KF_config.clock_phase_PSD = 1;

% Pseudo-range measurement noise SD (m)
TC_KF_config.pseudo_range_SD = 2.5;
% Pseudo-range rate measurement noise SD (m/s)
TC_KF_config.range_rate_SD = 0.1;

% Seeding of the random number generator for reproducability. Change 
% this value for a different random number sequence (may not work in Octave).
RandStream.setDefaultStream(RandStream('mt19937ar','seed',1));

% Begins

% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);

% End script if there is a problem with the file
if ~ok
    return;
end %if

% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
% simulation
[out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD] =...
    Tightly_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors...
    ,IMU_errors,GNSS_config,TC_KF_config);

% Plot the input motion profile and the errors (may not work in Octave).
close all;
Plot_profile(in_profile);
Plot_errors(out_errors);

% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);

% Ends