%Inertial_Demo_1ECI
%SCRIPT Inertial navigation demonstration:
%   Profile_1 (60s artificial car motion with two 90 deg turns)
%   ECI-frame inertial navigation equations
%   Tactical-grade IMU error model
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% Created 3/4/12 by Paul Groves

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details
clear all;
clc;
% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;

% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_1.csv';
% Output motion profile and error filenames
output_profile_name = 'Inertial_Demo_1ECI_Profile.csv';
output_errors_name = 'Inertial_Demo_1ECI_Errors.csv';

% Position initialization error (m; N,E,D)位置初始误差
initialization_errors.delta_r_eb_n = [4;2;3];

%===========可以改，只影响第一个历元的误差。Apr.30,2016 17:10===========%
% initialization_errors.delta_r_eb_n = [0;0;0];%
%===========可以改，只影响第一个历元的误差。Apr.30,2016 17:10===========%

% Velocity initialization error (m/s; N,E,D)速度初始误差
initialization_errors.delta_v_eb_n = [0.05;-0.05;0.1];
% Attitude initialization error (deg, converted to rad; @N,E,D)姿态初始误差
initialization_errors.delta_eul_nb_n = [-0.05;0.04;1]*deg_to_rad; % rad

% Accelerometer biases (micro-g, converted to m/s^2; body axes)加速度计零偏
IMU_errors.b_a = [900;-1300;800] * micro_g_to_meters_per_second_squared;
% Gyro biases (deg/hour, converted to rad/sec; body axes)陀螺仪零偏 P.133
IMU_errors.b_g = [-9;13;-8] * deg_to_rad / 3600;
% Accelerometer scale factor and cross coupling errors (ppm, converted to
% unitless; body axes)加速度计比例因子和交叉耦合误差
IMU_errors.M_a = [500, -300, 200;...
                 -150, -600, 250;...
                 -250,  100, 450] * 1E-6;
% Gyro scale factor and cross coupling errors (ppm, converted to unitless;
% body axes)陀螺仪比例因子和交叉耦合误差 P.135
IMU_errors.M_g = [400, -300,  250;...
                    0, -300, -150;...
                    0,    0, -350] * 1E-6;             
% Gyro g-dependent biases (deg/hour/g, converted to rad-sec/m; body
% axes)陀螺仪g相关零偏 P.139
IMU_errors.G_g = [0.9, -1.1, -0.6;...
                 -0.5,  1.9, -1.6;...
                  0.3,  1.1, -1.3] * deg_to_rad / (3600 * 9.80665);             
% Accelerometer noise root PSD (micro-g per root Hz, converted to m s^-1.5)
% 加速度计随机噪声的功率密度方根
IMU_errors.accel_noise_root_PSD = 100 *...
    micro_g_to_meters_per_second_squared;
% Gyro noise root PSD (deg per root hour, converted to rad s^-0.5)陀螺仪随机
% 噪声的功率密度方根 P.138
IMU_errors.gyro_noise_root_PSD = 0.01 * deg_to_rad / 60;
% Accelerometer quantization level (m/s^2)量化因子的数量级
IMU_errors.accel_quant_level = 1E-2;
% Gyro quantization level (rad/s)量化因子的数量级 P.138
IMU_errors.gyro_quant_level = 2E-4;

% Seeding of the random number generator for reproducability. Change 
% this value for a different random number sequence (may not work in Octave).
% RandStream.setDefaultStream(RandStream('mt19937ar','seed',1));

% Begins

% Input truth motion profile from .csv format file开始读取文件
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);

% End script if there is a problem with the file 判断读取文件是否有错误
if ~ok
    return;
end %if

% ECI Inertial navigation simulation 开始惯导仿真
[out_profile,out_errors] = Inertial_navigation_ECI(in_profile,no_epochs,...
    initialization_errors,IMU_errors);

% Plot the input motion profile and the errors (may not work in Octave).
close all;
Plot_profile(in_profile);
Plot_errors(out_errors);

% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);

% Ends