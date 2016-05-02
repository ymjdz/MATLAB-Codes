function [out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD] =...
    Loosely_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors,...
    IMU_errors,GNSS_config,LC_KF_config)
%Loosely_coupled_INS_GNSS - Simulates inertial navigation using ECEF
% navigation equations and kinematic model, GNSS using a least-squares
% positioning algorithm, and loosely-coupled INS/GNSS integration. 
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   in_profile   True motion profile array
%   no_epochs    Number of epochs of profile data
%   initialization_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .delta_v_eb_n     velocity error resolved along NED (m/s)
%     .delta_eul_nb_n   attitude error as NED Euler angles (rad)
%   IMU_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .b_a              Accelerometer biases (m/s^2)
%     .b_g              Gyro biases (rad/s)
%     .M_a              Accelerometer scale factor and cross coupling errors
%     .M_g              Gyro scale factor and cross coupling errors            
%     .G_g              Gyro g-dependent biases (rad-sec/m)             
%     .accel_noise_root_PSD   Accelerometer noise root PSD (m s^-1.5)
%     .gyro_noise_root_PSD    Gyro noise root PSD (rad s^-0.5)
%     .accel_quant_level      Accelerometer quantization level (m/s^2)
%     .gyro_quant_level       Gyro quantization level (rad/s)
%   GNSS_config
%     .epoch_interval     Interval between GNSS epochs (s)
%     .init_est_r_ea_e    Initial estimated position (m; ECEF)
%     .no_sat             Number of satellites in constellation
%     .r_os               Orbital radius of satellites (m)
%     .inclination        Inclination angle of satellites (deg)
%     .const_delta_lambda Longitude offset of constellation (deg)
%     .const_delta_t      Timing offset of constellation (s)
%     .mask_angle         Mask angle (deg)
%     .SIS_err_SD         Signal in space error SD (m)
%     .zenith_iono_err_SD Zenith ionosphere error SD (m)
%     .zenith_trop_err_SD Zenith troposphere error SD (m)
%     .code_track_err_SD  Code tracking error SD (m)
%     .rate_track_err_SD  Range rate tracking error SD (m/s)
%     .rx_clock_offset    Receiver clock offset at time=0 (m)
%     .rx_clock_drift     Receiver clock drift at time=0 (m/s)
%   LC_KF_config
%     .init_att_unc           Initial attitude uncertainty per axis (rad)
%     .init_vel_unc           Initial velocity uncertainty per axis (m/s)
%     .init_pos_unc           Initial position uncertainty per axis (m)
%     .init_b_a_unc           Initial accel. bias uncertainty (m/s^2)
%     .init_b_g_unc           Initial gyro. bias uncertainty (rad/s)
%     .gyro_noise_PSD         Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD        Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD         Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD          Gyro bias random walk PSD (rad^2 s^-3)
%     .pos_meas_SD            Position measurement noise SD per axis (m)
%     .vel_meas_SD            Velocity measurement noise SD per axis (m/s)
%
% Outputs:
%   out_profile        Navigation solution as a motion profile array
%   out_errors         Navigation solution error array
%   out_IMU_bias_est   Kalman filter IMU bias estimate array
%   out_clock          GNSS Receiver clock estimate array
%   out_KF_SD          Output Kalman filter state uncertainties
%
% Format of motion profiles:
%  Column 1: time (sec)
%  Column 2: latitude (rad)
%  Column 3: longitude (rad)
%  Column 4: height (m)
%  Column 5: north velocity (m/s)
%  Column 6: east velocity (m/s)
%  Column 7: down velocity (m/s)
%  Column 8: roll angle of body w.r.t NED (rad)
%  Column 9: pitch angle of body w.r.t NED (rad)
%  Column 10: yaw angle of body w.r.t NED (rad)
%
% Format of error array:
%  Column 1: time (sec)
%  Column 2: north position error (m)
%  Column 3: east position error (m)
%  Column 4: down position error (m)
%  Column 5: north velocity error (m/s)
%  Column 6: east velocity error (m/s)
%  Column 7: down velocity (error m/s)
%  Column 8: attitude error about north (rad)
%  Column 9: attitude error about east (rad)
%  Column 10: attitude error about down = heading error  (rad)
%
% Format of output IMU biases array:
%  Column 1: time (sec)
%  Column 2: estimated X accelerometer bias (m/s^2)
%  Column 3: estimated Y accelerometer bias (m/s^2)
%  Column 4: estimated Z accelerometer bias (m/s^2)
%  Column 5: estimated X gyro bias (rad/s)
%  Column 6: estimated Y gyro bias (rad/s)
%  Column 7: estimated Z gyro bias (rad/s)
%
% Format of receiver clock array:
%  Column 1: time (sec)
%  Column 2: estimated clock offset (m)
%  Column 3: estimated clock drift (m/s)
%
% Format of KF state uncertainties array:
%  Column 1: time (sec)
%  Column 2: X attitude error uncertainty (rad)
%  Column 3: Y attitude error uncertainty (rad)
%  Column 4: Z attitude error uncertainty (rad)
%  Column 5: X velocity error uncertainty (m/s)
%  Column 6: Y velocity error uncertainty (m/s)
%  Column 7: Z velocity error uncertainty (m/s)
%  Column 8: X position error uncertainty (m)
%  Column 9: Y position error uncertainty (m)
%  Column 10: Z position error uncertainty (m)
%  Column 11: X accelerometer bias uncertainty (m/s^2)
%  Column 12: Y accelerometer bias uncertainty (m/s^2)
%  Column 13: Z accelerometer bias uncertainty (m/s^2)
%  Column 14: X gyro bias uncertainty (rad/s)
%  Column 15: Y gyro bias uncertainty (rad/s)
%  Column 16: Z gyro bias uncertainty (rad/s)
%  Column 17: clock offset uncertainty (m)
%  Column 18: clock drift uncertainty (m/s)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Initialize true navigation solution
old_time = in_profile(1,1);
true_L_b = in_profile(1,2);
true_lambda_b = in_profile(1,3);
true_h_b = in_profile(1,4);
true_v_eb_n = in_profile(1,5:7)';
true_eul_nb = in_profile(1,8:10)';
true_C_b_n = Euler_to_CTM(true_eul_nb)';
[old_true_r_eb_e,old_true_v_eb_e,old_true_C_b_e] =...
    NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

% Determine satellite positions and velocities
[sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(old_time,...
    GNSS_config);

% Initialize the GNSS biases. Note that these are assumed constant throughout 
% the simulation and are based on the initial elevation angles. Therefore, 
% this function is unsuited to simulations longer than about 30 min.
GNSS_biases = Initialize_GNSS_biases(sat_r_es_e,old_true_r_eb_e,true_L_b,...
    true_lambda_b,GNSS_config);

% Generate GNSS measurements
[GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(old_time,...
    sat_r_es_e,sat_v_es_e,old_true_r_eb_e,true_L_b,true_lambda_b,...
    old_true_v_eb_e,GNSS_biases,GNSS_config);

% Determine Least-squares GNSS position solution and use to initialize INS
[GNSS_r_eb_e,GNSS_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
    GNSS_measurements,no_GNSS_meas,GNSS_config.init_est_r_ea_e,[0;0;0]);
old_est_r_eb_e = GNSS_r_eb_e;
old_est_v_eb_e = GNSS_v_eb_e;
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =...
    pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;

% Initialize estimated attitude solution
old_est_C_b_n = Initialize_NED_attitude(true_C_b_n,initialization_errors);
[temp1,temp2,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,...
    old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

% Initialize output profile record and errors record
out_profile = zeros(no_epochs,10);
out_errors = zeros(no_epochs,10);

% Generate output profile record
out_profile(1,1) = old_time;
out_profile(1,2) = old_est_L_b;
out_profile(1,3) = old_est_lambda_b;
out_profile(1,4) = old_est_h_b;
out_profile(1,5:7) = old_est_v_eb_n';
out_profile(1,8:10) = CTM_to_Euler(old_est_C_b_n')';

% Determine errors and generate output record
[delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
    old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n,...
    true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
out_errors(1,1) = old_time;
out_errors(1,2:4) = delta_r_eb_n';
out_errors(1,5:7) = delta_v_eb_n';
out_errors(1,8:10) = delta_eul_nb_n';

% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_LC_P_matrix(LC_KF_config);
est_IMU_bias = zeros(6,1);

% Initialize IMU quantization residuals
quant_residuals = [0;0;0;0;0;0];

% Generate IMU bias and clock output records
out_IMU_bias_est(1,1) = old_time;
out_IMU_bias_est(1,2:7) = est_IMU_bias';
out_clock(1,1) = old_time;
out_clock(1,2:3) = est_clock;

% Generate KF uncertainty record
out_KF_SD(1,1) = old_time;
for i =1:15
    out_KF_SD(1,i+1) = sqrt(P_matrix(i,i));
end % for i

% Initialize GNSS model timing
time_last_GNSS = old_time;
GNSS_epoch = 1;

% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;

% Main loop
for epoch = 2:no_epochs

    % Update progress bar
    if (epoch - progress_epoch) > (no_epochs/20)
        progress_mark = progress_mark + 1;
        progress_epoch = epoch;
        fprintf(strcat(rewind,bars(1:progress_mark),...
            dots(1:(20 - progress_mark))));
    end % if epoch    
    
    % Input data from motion profile
    time = in_profile(epoch,1);
    true_L_b = in_profile(epoch,2);
    true_lambda_b = in_profile(epoch,3);
    true_h_b = in_profile(epoch,4);
    true_v_eb_n = in_profile(epoch,5:7)';
    true_eul_nb = in_profile(epoch,8:10)';
    true_C_b_n = Euler_to_CTM(true_eul_nb)';
    [true_r_eb_e,true_v_eb_e,true_C_b_e] =...
        NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

    % Time interval
    tor_i = time - old_time;
    
    % Calculate specific force and angular rate
    [true_f_ib_b,true_omega_ib_b] = Kinematics_ECEF(tor_i,true_C_b_e,...
        old_true_C_b_e,true_v_eb_e,old_true_v_eb_e,old_true_r_eb_e);
        
     % Simulate IMU errors
    [meas_f_ib_b,meas_omega_ib_b,quant_residuals] = IMU_model(tor_i,...
        true_f_ib_b,true_omega_ib_b,IMU_errors,quant_residuals);
    
    % Correct IMU errors
    meas_f_ib_b = meas_f_ib_b - est_IMU_bias(1:3);
    meas_omega_ib_b = meas_omega_ib_b - est_IMU_bias(4:6);
    
    % Update estimated navigation solution
    [est_r_eb_e,est_v_eb_e,est_C_b_e] = Nav_equations_ECEF(tor_i,...
        old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,meas_f_ib_b,...
        meas_omega_ib_b);

    % Determine whether to update GNSS simulation and run Kalman filter
    if (time - time_last_GNSS) >= GNSS_config.epoch_interval
        GNSS_epoch = GNSS_epoch + 1;
        tor_s = time - time_last_GNSS;  % KF time interval
        time_last_GNSS = time;
   
        % Determine satellite positions and velocities
        [sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(time,...
            GNSS_config);

        % Generate GNSS measurements
        [GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(...
            time,sat_r_es_e,sat_v_es_e,true_r_eb_e,true_L_b,true_lambda_b,...
            true_v_eb_e,GNSS_biases,GNSS_config);

        % Determine GNSS position solution
        [GNSS_r_eb_e,GNSS_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
            GNSS_measurements,no_GNSS_meas,GNSS_r_eb_e,GNSS_v_eb_e);

        % Run Integration Kalman filter
        [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
            LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
            est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,...
            est_L_b,LC_KF_config);

        % Generate IMU bias and clock output records
        out_IMU_bias_est(GNSS_epoch,1) = time;
        out_IMU_bias_est(GNSS_epoch,2:7) = est_IMU_bias';
        out_clock(GNSS_epoch,1) = time;
        out_clock(GNSS_epoch,2:3) = est_clock;

        % Generate KF uncertainty output record
        out_KF_SD(GNSS_epoch,1) = time;
        for i =1:15
            out_KF_SD(GNSS_epoch,i+1) = sqrt(P_matrix(i,i));
        end % for i

    end % if time    
    
    % Convert navigation solution to NED
    [est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);

    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2) = est_L_b;
    out_profile(epoch,3) = est_lambda_b;
    out_profile(epoch,4) = est_h_b;
    out_profile(epoch,5:7) = est_v_eb_n';
    out_profile(epoch,8:10) = CTM_to_Euler(est_C_b_n')';
    
    % Determine errors and generate output record
    [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
        est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
        true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
    out_errors(epoch,1) = time;
    out_errors(epoch,2:4) = delta_r_eb_n';
    out_errors(epoch,5:7) = delta_v_eb_n';
    out_errors(epoch,8:10) = delta_eul_nb_n';
    
    % Reset old values
    old_time = time;
    old_true_r_eb_e = true_r_eb_e;
    old_true_v_eb_e = true_v_eb_e;
    old_true_C_b_e = true_C_b_e;
    old_est_r_eb_e = est_r_eb_e;
    old_est_v_eb_e = est_v_eb_e;
    old_est_C_b_e = est_C_b_e;

end %epoch

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));

% Ends