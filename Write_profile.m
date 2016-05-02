function Write_profile(filename,out_profile)
%Write_profile - outputs a motion profile in the following .csv format
% Column 1: time (sec)
% Column 2: latitude (deg)
% Column 3: longitude (deg)
% Column 4: height (m)
% Column 5: north velocity (m/s)
% Column 6: east velocity (m/s)
% Column 7: down velocity (m/s)
% Column 8: roll angle of body w.r.t NED (deg)
% Column 9: pitch angle of body w.r.t NED (deg)
% Column 10: yaw angle of body w.r.t NED (deg)
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 31/3/2012 by Paul Groves
%
% Inputs:
%   filename     Name of file to write
%   out_profile  Array of data to write

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Parameters
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;

% Convert output profile from radians to degrees
out_profile(:,2:3) = rad_to_deg * out_profile(:,2:3);
out_profile(:,8:10) = rad_to_deg * out_profile(:,8:10);

% Write output profile
dlmwrite(filename,out_profile,'newline','pc','precision',12);

% Ends