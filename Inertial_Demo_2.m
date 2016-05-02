%Inertial_Demo_2
%SCRIPT Inertial navigation demonstration:
%   Profile_1 (60s artificial car motion with two 90 deg turns)
%   Local-navigation-frame inertial navigation equations
%   Aviation-grade IMU error model
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% Created 4/4/12 by Paul Groves

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
output_profile_name = 'Inertial_Demo_2_Profile.csv';
output_errors_name = 'Inertial_Demo_2_Errors.csv';

% Position initialization error (m; N,E,D)
initialization_errors.delta_r_eb_n = [4;2;3];
% Velocity initialization error (m/s; N,E,D)
initialization_errors.delta_v_eb_n = [0.05;-0.05;0.1];
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
[out_profile,out_errors] = Inertial_navigation_NED(in_profile,no_epochs,...
    initialization_errors,IMU_errors);

% Plot the input motion profile and the errors (may not work in Octave).
close all;
Plot_profile(in_profile);
Plot_errors(out_errors);

% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);

% Ends