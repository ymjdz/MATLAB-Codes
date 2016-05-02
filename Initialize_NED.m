function [est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
    Initialize_NED(L_b,lambda_b,h_b,v_eb_n,C_b_n,initialization_errors)
%Initialize_NED - Initializes the curvilinear position, velocity, and
%attitude solution by adding errors to the truth.
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 3/4/2012 by Paul Groves
%
% Inputs:
%   L_b           true latitude (rad)
%   lambda_b      true longitude (rad)
%   h_b           true height (m)
%   v_eb_n        true velocity of body frame w.r.t. ECEF frame, resolved
%                 along north, east, and down (m/s)
%   C_b_n         true body-to-NED coordinate transformation matrix
%   initialization_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .delta_v_eb_n     velocity error resolved along NED (m/s)
%     .delta_eul_nb_n   attitude error as NED Euler angles (rad)
%
% Outputs:
%   est_L_b       latitude solution (rad)
%   est_lambda_b  longitude solution (rad)
%   est_h_b       height solution (m)
%   est_v_eb_n    velocity solution of body frame w.r.t. ECEF frame,
%                 resolved along north, east, and down (m/s)
%   est_C_b_n     body-to-NED coordinate transformation matrix solution

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Position initialization, using (2.119)
[R_N,R_E] = Radii_of_curvature(L_b);
est_L_b = L_b + initialization_errors.delta_r_eb_n(1) / (R_N + h_b);
est_lambda_b = lambda_b + initialization_errors.delta_r_eb_n(2) /...
    ((R_E + h_b) * cos(L_b));
est_h_b = h_b - initialization_errors.delta_r_eb_n(3);

% Velocity initialization
est_v_eb_n = v_eb_n + initialization_errors.delta_v_eb_n;

% Attitude initialization, using (5.109) and (5.111)
delta_C_b_n = Euler_to_CTM(-initialization_errors.delta_eul_nb_n);
est_C_b_n = delta_C_b_n * C_b_n;
    
% Ends