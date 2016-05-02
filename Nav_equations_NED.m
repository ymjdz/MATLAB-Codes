function [L_b,lambda_b,h_b,v_eb_n,C_b_n] = Nav_equations_NED(tor_i,...
        old_L_b,old_lambda_b,old_h_b,old_v_eb_n,old_C_b_n,f_ib_b,omega_ib_b)
%Nav_equations_NED - Runs precision local-navigation-frame inertial 
%navigation equations (Note: only the attitude update and specific force
%frame transformation phases are precise.)
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   old_L_b       previous latitude (rad)
%   old_lambda_b  previous longitude (rad)
%   old_h_b       previous height (m)
%   old_C_b_n     previous body-to-NED coordinate transformation matrix
%   old_v_eb_n    previous velocity of body frame w.r.t. ECEF frame, resolved
%                 along north, east, and down (m/s)
%   f_ib_b        specific force of body frame w.r.t. ECEF frame, resolved
%                 along body-frame axes, averaged over time interval (m/s^2)
%   omega_ib_b    angular rate of body frame w.r.t. ECEF frame, resolved
%                 about body-frame axes, averaged over time interval (rad/s)
% Outputs:
%   L_b           latitude (rad)
%   lambda_b      longitude (rad)
%   h_b           height (m)
%   v_eb_n        velocity of body frame w.r.t. ECEF frame, resolved along
%                 north, east, and down (m/s)
%   C_b_n         body-to-NED coordinate transformation matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% parameters
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Begins

% PRELIMINARIES
% Calculate attitude increment, magnitude, and skew-symmetric matrix
alpha_ib_b = omega_ib_b * tor_i;
mag_alpha = sqrt(alpha_ib_b' * alpha_ib_b);
Alpha_ib_b = Skew_symmetric(alpha_ib_b);  

% From (2.123) , determine the angular rate of the ECEF frame
% w.r.t the ECI frame, resolved about NED
omega_ie_n = omega_ie * [cos(old_L_b); 0; - sin(old_L_b)];
    
% From (5.44), determine the angular rate of the NED frame
% w.r.t the ECEF frame, resolved about NED
[old_R_N,old_R_E] = Radii_of_curvature(old_L_b);
old_omega_en_n = [old_v_eb_n(2) / (old_R_E + old_h_b);...
    -old_v_eb_n(1) / (old_R_N + old_h_b);...
    -old_v_eb_n(2) * tan(old_L_b) / (old_R_E + old_h_b)];
    
% SPECIFIC FORCE FRAME TRANSFORMATION
% Calculate the average body-to-ECEF-frame coordinate transformation
% matrix over the update interval using (5.84) and (5.86)
if mag_alpha>1.E-8
    ave_C_b_n = old_C_b_n * (eye(3) + (1 - cos(mag_alpha)) / mag_alpha^2 ...
        * Alpha_ib_b + (1 - sin(mag_alpha) / mag_alpha) / mag_alpha^2 ...
        * Alpha_ib_b * Alpha_ib_b) -...
        0.5 * Skew_symmetric(old_omega_en_n + omega_ie_n) * old_C_b_n;
else
     ave_C_b_n = old_C_b_n -...
         0.5 * Skew_symmetric(old_omega_en_n + omega_ie_n) * old_C_b_n;
end %if mag_alpha     

% Transform specific force to ECEF-frame resolving axes using (5.86)
f_ib_n = ave_C_b_n * f_ib_b;
    
% UPDATE VELOCITY
% From (5.54),
v_eb_n = old_v_eb_n + tor_i * (f_ib_n + Gravity_NED(old_L_b,old_h_b) -...
    Skew_symmetric(old_omega_en_n + 2 * omega_ie_n) * old_v_eb_n);

% UPDATE CURVILINEAR POSITION
% Update height using (5.56)
h_b = old_h_b - 0.5 * tor_i * (old_v_eb_n(3) + v_eb_n(3));

% Update latitude using (5.56)
L_b = old_L_b + 0.5 * tor_i * (old_v_eb_n(1) / (old_R_N + old_h_b) +...
    v_eb_n(1) / (old_R_N + h_b));

% Calculate meridian and transverse radii of curvature
[R_N,R_E]= Radii_of_curvature(L_b);

% Update longitude using (5.56)
lambda_b = old_lambda_b + 0.5 * tor_i * (old_v_eb_n(2) / ((old_R_E +...
    old_h_b) * cos(old_L_b)) + v_eb_n(2) / ((R_E + h_b) * cos(L_b))); 

% ATTITUDE UPDATE
% From (5.44), determine the angular rate of the NED frame
% w.r.t the ECEF frame, resolved about NED
omega_en_n = [v_eb_n(2) / (R_E + h_b);...
    -v_eb_n(1) / (R_N + h_b);...
    -v_eb_n(2) * tan(L_b) / (R_E + h_b)];
                       
% Obtain coordinate transformation matrix from the new attitude w.r.t. an
% inertial frame to the old using Rodrigues' formula, (5.73)
if mag_alpha>1.E-8
    C_new_old = eye(3) + sin(mag_alpha) / mag_alpha * Alpha_ib_b +...
        (1 - cos(mag_alpha)) / mag_alpha^2 * Alpha_ib_b * Alpha_ib_b;
else
    C_new_old = eye(3) + Alpha_ib_b;
end %if mag_alpha    
    
% Update attitude using (5.77)
C_b_n = (eye(3) - Skew_symmetric(omega_ie_n + 0.5 * omega_en_n + 0.5 *...
    old_omega_en_n)  * tor_i) * old_C_b_n * C_new_old;
    
% Ends