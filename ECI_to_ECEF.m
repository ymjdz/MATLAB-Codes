function [r_eb_e,v_eb_e,C_b_e] = ECI_to_ECEF(t,r_ib_i,v_ib_i,C_b_i)
%ECI_to_ECEF - Converts position, velocity, and attitude from ECI- to
%ECEF-frame referenced and resolved
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs:
%   t             time (s)
%   r_ib_i        Cartesian position of body frame w.r.t. ECI frame, resolved
%                 along ECI-frame axes (m)
%   v_ib_i        velocity of body frame w.r.t. ECI frame, resolved along
%                 ECI-frame axes (m/s)
%   C_b_i         body-to-ECI-frame coordinate transformation matrix
%
% Outputs:
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix

% Parameters
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Calculate ECI to ECEF coordinate transformation matrix using (2.145)
C_i_e = [cos(omega_ie * t), sin(omega_ie * t), 0;...
        -sin(omega_ie * t), cos(omega_ie * t), 0;...
                         0,                 0, 1];

% Transform position using (2.146)                   
r_eb_e = C_i_e * r_ib_i;

% Transform velocity using (2.145)
v_eb_e = C_i_e * (v_ib_i - omega_ie * [-r_ib_i(2);r_ib_i(1);0]);

% Transform attitude using (2.15)
C_b_e = C_i_e * C_b_i;

% Ends