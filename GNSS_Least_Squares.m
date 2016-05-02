function [out_profile,out_errors,out_clock] = GNSS_Least_Squares(in_profile,...
    no_epochs,GNSS_config)
%GNSS_Least_Squares - Simulates stand-alone GNSS using a least-squares
%positioning algorithm
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   in_profile   True motion profile array
%   no_epochs    Number of epochs of profile data
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
%
% Outputs:
%   out_profile   Navigation solution as a motion profile array
%   out_errors    Navigation solution error array
%   out_clock     Receiver clock estimate array
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
%  Column 7: down velocity error (m/s)
%  Column 8: NOT USED (attitude error about north (rad))
%  Column 9: NOT USED (attitude error about east (rad))
%  Column 10: NOT USED (attitude error about down = heading error  (rad))
%
% Format of receiver clock array:
%  Column 1: time (sec)
%  Column 2: estimated clock offset (m)
%  Column 3: estimated clock drift (m/s)

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
[true_r_eb_e,true_v_eb_e] =...
    pv_NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n);

time_last_GNSS = old_time;
GNSS_epoch = 1;

% Determine satellite positions and velocities
[sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(old_time,...
    GNSS_config);

% Initialize the GNSS biases. Note that these are assumed constant throughout 
% the simulation and are based on the initial elevation angles. Therefore, 
% this function is unsuited to simulations longer than about 30 min.
GNSS_biases = Initialize_GNSS_biases(sat_r_es_e,true_r_eb_e,true_L_b,...
    true_lambda_b,GNSS_config);

% Generate GNSS measurements
[GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(old_time,...
    sat_r_es_e,sat_v_es_e,true_r_eb_e,true_L_b,true_lambda_b,true_v_eb_e,...
    GNSS_biases,GNSS_config);

% Determine GNSS position solution
[est_r_eb_e,est_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
    GNSS_measurements,no_GNSS_meas,GNSS_config.init_est_r_ea_e,[0;0;0]);

est_C_b_n = true_C_b_n; % This sets the attitude errors to zero


[est_L_b,est_lambda_b,est_h_b,est_v_eb_n] =...
    pv_ECEF_to_NED(est_r_eb_e,est_v_eb_e);

% Generate output profile record
out_profile(1,1) = old_time;
out_profile(1,2) = est_L_b;
out_profile(1,3) = est_lambda_b;
out_profile(1,4) = est_h_b;
out_profile(1,5:7) = est_v_eb_n';
out_profile(1,8:10) = CTM_to_Euler(est_C_b_n')';

% Determine errors and generate output record
[delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
    est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
    true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
out_errors(1,1) = old_time;
out_errors(1,2:4) = delta_r_eb_n';
out_errors(1,5:7) = delta_v_eb_n';
out_errors(1,8:10) = [0;0;0];

% Generate clock output record
out_clock(1,1) = old_time;
out_clock(1,2:3) = est_clock(1:2);

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
    
   % Input time from motion profile
   time = in_profile(epoch,1);
        
    % Determine whether to update GNSS simulation
    if (time - time_last_GNSS) >= GNSS_config.epoch_interval
        GNSS_epoch = GNSS_epoch + 1;
        time_last_GNSS = time;
    
        % Input data from motion profile
        true_L_b = in_profile(epoch,2);
        true_lambda_b = in_profile(epoch,3);
        true_h_b = in_profile(epoch,4);
        true_v_eb_n = in_profile(epoch,5:7)';
        true_eul_nb = in_profile(epoch,8:10)';
        true_C_b_n = Euler_to_CTM(true_eul_nb)';
        [true_r_eb_e,true_v_eb_e] =...
            pv_NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n);

        % Determine satellite positions and velocities
        [sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(time,...
            GNSS_config);

        % Generate GNSS measurements
        [GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(...
            time,sat_r_es_e,sat_v_es_e,true_r_eb_e,true_L_b,true_lambda_b,...
            true_v_eb_e,GNSS_biases,GNSS_config);

        % Determine GNSS position solution
        [est_r_eb_e,est_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
            GNSS_measurements,no_GNSS_meas,est_r_eb_e,est_v_eb_e);
        [est_L_b,est_lambda_b,est_h_b,est_v_eb_n] =...
            pv_ECEF_to_NED(est_r_eb_e,est_v_eb_e);

        est_C_b_n = true_C_b_n; % This sets the attitude errors to zero

        % Generate output profile record
        out_profile(GNSS_epoch,1) = time;
        out_profile(GNSS_epoch,2) = est_L_b;
        out_profile(GNSS_epoch,3) = est_lambda_b;
        out_profile(GNSS_epoch,4) = est_h_b;
        out_profile(GNSS_epoch,5:7) = est_v_eb_n';
        out_profile(GNSS_epoch,8:10) = CTM_to_Euler(est_C_b_n')';
    
        % Determine errors and generate output record
        [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
            est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
            true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
        out_errors(GNSS_epoch,1) = time;
        out_errors(GNSS_epoch,2:4) = delta_r_eb_n';
        out_errors(GNSS_epoch,5:7) = delta_v_eb_n';
        out_errors(GNSS_epoch,8:10) = [0;0;0];
    
        % Generate clock output record
        out_clock(GNSS_epoch,1) = time;
        out_clock(GNSS_epoch,2:3) = est_clock(1:2);

        % Reset old values
        old_time = time;
    end % if time    

end %epoch

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));

% Ends