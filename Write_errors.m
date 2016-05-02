function Write_errors(filename,out_errors)
%Write_errors - outputs the errors in the following .csv format
% Column 1: time (sec)
% Column 2: north position error (m)
% Column 3: east position error (m)
% Column 4: down position error (m)
% Column 5: north velocity (m/s)
% Column 6: east velocity (m/s)
% Column 7: down velocity (m/s)
% Column 8: roll component of NED attitude error (deg)
% Column 9: pitch component of NED attitude error (deg)
% Column 10: yaw component of NED attitude error (deg)
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 31/3/2012 by Paul Groves
%
% Inputs:
%   filename     Name of file to write
%   out_errors   Array of data to write

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Parameters
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;

% Convert attitude errors from radians to degrees
out_errors(:,8:10) = rad_to_deg * out_errors(:,8:10);

% Write output profile
dlmwrite(filename,out_errors,'newline','pc','precision',12);

% Ends