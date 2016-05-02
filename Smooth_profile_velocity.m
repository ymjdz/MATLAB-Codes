function Smooth_profile_velocity(in_filename,out_filename)
%Smooth_profile_velocity - Adjusts the velocity in a motion profile file to
% remove jitter resulting from numerical rounding in the position input to
% Adjust_profile_velocity
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs
%  in_filename     Name of inpit file, e.g. 'In_profile.csv'
%  out_filename    Name of inpit file, e.g. 'Out_profile.csv'

% Begins

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Parameters
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;


% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(in_filename);

% End script if there is a problem with the file
if ~ok
    return;
end %if

% First epoch
out_profile(1,:) = in_profile(1,:);

% Main loop
for epoch = 2:(no_epochs-1)

    % Input data from profile
    time = in_profile(epoch,1);
    L_b = in_profile(epoch,2);
    lambda_b = in_profile(epoch,3);
    h_b = in_profile(epoch,4);
    v_eb_n_current = in_profile(epoch,5:7)';
    eul_nb = in_profile(epoch,8:10)';
    v_eb_n_prev = in_profile(epoch - 1,5:7)';
    v_eb_n_next = in_profile(epoch + 1,5:7)';
 
    % Smooth velocity
    v_eb_n = 0.5 * v_eb_n_current + 0.25 * v_eb_n_prev + 0.25 * v_eb_n_next;
    
    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2) = L_b;
    out_profile(epoch,3) = lambda_b;
    out_profile(epoch,4) = h_b;
    out_profile(epoch,5:7) = v_eb_n';
    out_profile(epoch,8:10) = eul_nb';
    

end %epoch

% Last epoch
out_profile(no_epochs,:) = in_profile(no_epochs,:);


% Write output profile
Write_profile(out_filename,out_profile);

% Ends