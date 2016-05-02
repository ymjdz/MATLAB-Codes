function Adjust_profile_position(in_filename,out_filename)
%Adjust_profile_position - Adjusts the position in a motion profile file to
% make it consistent with the velocity
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs
%  in_filename     Name of inpit file, e.g. 'In_profile.csv'
%  out_filename    Name of inpit file, e.g. 'Out_profile.csv'

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Parameters
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;


% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(in_filename);

% End script if there is a problem with the file
if ~ok
    return;
end %if

% Initialize navigation solution
old_time = in_profile(1,1);
old_L_b = in_profile(1,2);
old_lambda_b = in_profile(1,3);
old_h_b = in_profile(1,4);
old_v_eb_n = in_profile(1,5:7)';
old_eul_nb = in_profile(1,8:10)';
out_profile(1,:) = in_profile(1,:);

% Main loop
for epoch = 2:no_epochs

    % Input data from profile
    time = in_profile(epoch,1);
    v_eb_n = in_profile(epoch,5:7)';
    eul_nb = in_profile(epoch,8:10)';
    
    % Time interval
    tor_i = time - old_time;
    
    % Update position 
    [L_b,lambda_b,h_b]= Update_curvilinear_position(tor_i,old_L_b,...
    old_lambda_b,old_h_b,old_v_eb_n,v_eb_n);
    
    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2) = L_b;
    out_profile(epoch,3) = lambda_b;
    out_profile(epoch,4) = h_b;
    out_profile(epoch,5:7) = v_eb_n';
    out_profile(epoch,8:10) = eul_nb';
    
    % Reset old values
    old_time = time;
    old_L_b = L_b;
    old_lambda_b = lambda_b;
    old_h_b = h_b;
    old_v_eb_n = v_eb_n;
    old_eul_nb = eul_nb;

end %epoch

% Write output profile
Write_profile(out_filename,out_profile);

% Ends