function [r_ib_i,v_ib_i,C_b_i] = Nav_equations_ECI(tor_i,old_r_ib_i,...
        old_v_ib_i,old_C_b_i,f_ib_b,omega_ib_b)
%Nav_equations_ECI - Runs precision ECI-frame inertial navigation
%equations
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   old_r_ib_i    previous Cartesian position of body frame w.r.t. ECI
%                 frame, resolved along ECI-frame axes (m)
%   old_C_b_i     previous body-to-ECI-frame coordinate transformation matrix
%   old_v_ib_i    previous velocity of body frame w.r.t. ECI frame, resolved
%                 along ECI-frame axes (m/s)
%   f_ib_b        specific force of body frame w.r.t. ECI frame, resolved
%                 along body-frame axes, averaged over time interval (m/s^2)
%   omega_ib_b    angular rate of body frame w.r.t. ECI frame, resolved
%                 about body-frame axes, averaged over time interval (rad/s)
% Outputs:
%   r_ib_i        Cartesian position of body frame w.r.t. ECI frame, resolved
%                 along ECI-frame axes (m)
%   v_ib_i        velocity of body frame w.r.t. ECI frame, resolved along
%                 ECI-frame axes (m/s)
%   C_b_i         body-to-ECI-frame coordinate transformation matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% ATTITUDE UPDATE
% Calculate attitude increment, magnitude, and skew-symmetric matrix
alpha_ib_b = omega_ib_b * tor_i;
mag_alpha = sqrt(alpha_ib_b' * alpha_ib_b);
Alpha_ib_b = Skew_symmetric(alpha_ib_b);  

% Obtain coordinate transformation matrix from the new attitude to the old
% using Rodrigues' formula, (5.73)
if mag_alpha>1.E-8
    C_new_old = eye(3) + sin(mag_alpha) / mag_alpha * Alpha_ib_b +...
        (1 - cos(mag_alpha)) / mag_alpha^2 * Alpha_ib_b * Alpha_ib_b;
else
    C_new_old = eye(3) + Alpha_ib_b;
end %if mag_alpha    

% Update attitude
C_b_i =  old_C_b_i * C_new_old;
    
% SPECIFIC FORCE FRAME TRANSFORMATION
% Calculate the average body-to-ECI-frame coordinate transformation
% matrix over the update interval using (5.84)
if mag_alpha>1.E-8
    ave_C_b_i = old_C_b_i * (eye(3) + (1 - cos(mag_alpha)) / mag_alpha^2 ...
        * Alpha_ib_b + (1 - sin(mag_alpha) / mag_alpha) / mag_alpha^2 ...
        * Alpha_ib_b * Alpha_ib_b);
else
     ave_C_b_i = old_C_b_i;
end %if mag_alpha     

% Transform specific force to ECI-frame resolving axes using (5.81)
f_ib_i = ave_C_b_i * f_ib_b;
    
% UPDATE VELOCITY
% From (5.18) and (5.20),
v_ib_i = old_v_ib_i + tor_i * (f_ib_i + Gravitation_ECI(old_r_ib_i));

% UPDATE CARTESIAN POSITION
% From (5.23),
r_ib_i = old_r_ib_i + (v_ib_i + old_v_ib_i) * 0.5 * tor_i; 

% Ends