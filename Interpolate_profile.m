function Interpolate_profile(input_profile_name,output_profile_name)
%Interpolate_profile - Interpolating .csv motion profiles:
%  
% Inputs
% input_profile_name     Input motion profile filename
% output_profile_name    Output motion profile filename
%
% Created 3/4/12 by Paul Groves

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% CONFIGURATION
% Jitter standard deviation on interpolated velocity (m/s)
Velocity_jitter = 0.001;
% Jitter standard deviation on interpolated attitude (rad)
Attitude_jitter = 0.00002;

% Seeding of the random number generator for reproducability. Change 
% this value for a different random number sequence.
rng(1);

% Begins

% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);

% End script if there is a problem with the file
if ~ok
    return;
end %if

% Initialize true navigation solution
old_time = in_profile(1,1);
old_true_L_b = in_profile(1,2);
old_true_lambda_b = in_profile(1,3);
old_true_h_b = in_profile(1,4);
old_true_v_eb_n = in_profile(1,5:7)';
old_true_eul_nb = in_profile(1,8:10)';

out_profile(1,:)=in_profile(1,:);

% Main loop
for epoch = 2:no_epochs

    % Input data from motion profile
    time = in_profile(epoch,1);
    true_L_b = in_profile(epoch,2);
    true_lambda_b = in_profile(epoch,3);
    true_h_b = in_profile(epoch,4);
    true_v_eb_n = in_profile(epoch,5:7)';
    true_eul_nb = in_profile(epoch,8:10)';  

    % Time interval
    tor_i = time - old_time;
    
    % Interpolate
    inter_L_b = 0.5 * (true_L_b + old_true_L_b);
    inter_lambda_b = 0.5 * (true_lambda_b + old_true_lambda_b);
    inter_h_b = 0.5 * (true_h_b + old_true_h_b);
    inter_v_eb_n = 0.5 * (true_v_eb_n + old_true_v_eb_n) +...
        randn(3,1) * Velocity_jitter;
    inter_eul_nb = 0.5 * (true_eul_nb + old_true_eul_nb) +...
        randn(3,1) * Attitude_jitter;
   
    % Generate output profile records
    epoch1 = 2 * epoch -2;
    epoch2 = 2 * epoch -1;
    out_profile(epoch1,1) = 0.5* (time + old_time);
    out_profile(epoch1,2) = inter_L_b;
    out_profile(epoch1,3) = inter_lambda_b;
    out_profile(epoch1,4) = inter_h_b;
    out_profile(epoch1,5:7) = inter_v_eb_n';
    out_profile(epoch1,8:10) = inter_eul_nb';
    out_profile(epoch2,1) = time;
    out_profile(epoch2,2) = true_L_b;
    out_profile(epoch2,3) = true_lambda_b;
    out_profile(epoch2,4) = true_h_b;
    out_profile(epoch2,5:7) = true_v_eb_n';
    out_profile(epoch2,8:10) = true_eul_nb';
    
    % Reset old values
    old_time = time;
    old_true_L_b = true_L_b;
    old_true_lambda_b = true_lambda_b;
    old_true_h_b = true_h_b;
    old_true_v_eb_n = true_v_eb_n;
    old_true_eul_nb = true_eul_nb;

end %epoch

% Write output profile
Write_profile(output_profile_name,out_profile);

% Ends