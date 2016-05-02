%GNSS_Demo_2
%SCRIPT Stand-alone GNSS demo with Kalman filter solution:
%   Profile_1 (60s artificial car motion with two 90 deg turns)
%
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% Created 12/4/12 by Paul Groves

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;

% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_1.csv';
% Output motion profile and error filenames
output_profile_name = 'GNSS_Demo_2_Profile.csv';
output_errors_name = 'GNSS_Demo_2_Errors.csv';

% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 1;

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

% Initial position uncertainty per axis (m)
GNSS_KF_config.init_pos_unc = 10;
% Initial velocity uncertainty per axis (m/s)
GNSS_KF_config.init_vel_unc = 0.1;
% Initial clock offset uncertainty per axis (m)
GNSS_KF_config.init_clock_offset_unc = 10;
% Initial clock drift uncertainty per axis (m/s)
GNSS_KF_config.init_clock_drift_unc = 0.1;

% Acceleration PSD per axis (m^2/s^3)
GNSS_KF_config.accel_PSD = 10;
% Receiver clock frequency-drift PSD (m^2/s^3)
GNSS_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
GNSS_KF_config.clock_phase_PSD = 1;

% Pseudo-range measurement noise SD (m)
GNSS_KF_config.pseudo_range_SD = 2.5;
% Pseudo-range rate measurement noise SD (m/s)
GNSS_KF_config.range_rate_SD = 0.05;

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

% NED Inertial navigation simulation
[out_profile,out_errors,out_clock,out_KF_SD] = GNSS_Kalman_Filter(...
    in_profile,no_epochs,GNSS_config,GNSS_KF_config);

% Plot the input motion profile and the errors (may not work in Octave).
close all;
Plot_profile(in_profile);
Plot_errors(out_errors);

% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);

% Ends