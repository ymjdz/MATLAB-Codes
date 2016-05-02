function  [f_ib_b,omega_ib_b] = Kinematics_NED(tor_i,C_b_n,old_C_b_n,...
        v_eb_n,old_v_eb_n,L_b,h_b,old_L_b,old_h_b)
%Kinematics_NED - calculates specific force and angular rate from input
%w.r.t and resolved along north, east, and down
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   C_b_n         body-to-NED coordinate transformation matrix
%   old_C_b_n     previous body-to-NED coordinate transformation matrix
%   v_eb_n        velocity of body frame w.r.t. ECEF frame, resolved along
%                 north, east, and down (m/s)
%   old_v_eb_n    previous velocity of body frame w.r.t. ECEF frame, resolved
%                 along north, east, and down (m/s)
%   L_b           latitude (rad)
%   h_b           height (m)
%   old_L_b       previous latitude (rad)
%   old_h_b       previous height (m)
% Outputs:
%   f_ib_b        specific force of body frame w.r.t. ECEF frame, resolved
%                 along body-frame axes, averaged over time interval (m/s^2)
%   omega_ib_b    angular rate of body frame w.r.t. ECEF frame, resolved
%                 about body-frame axes, averaged over time interval (rad/s)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Parameters
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Begins

if tor_i > 0

    % From (2.123) , determine the angular rate of the ECEF frame
    % w.r.t the ECI frame, resolved about NED
    omega_ie_n = omega_ie * [cos(old_L_b); 0; - sin(old_L_b)];
    
    % From (5.44), determine the angular rate of the NED frame
    % w.r.t the ECEF frame, resolved about NED
    [old_R_N,old_R_E] = Radii_of_curvature(old_L_b);
    [R_N,R_E] = Radii_of_curvature(L_b);
    old_omega_en_n = [old_v_eb_n(2) / (old_R_E + old_h_b);...
        -old_v_eb_n(1) / (old_R_N + old_h_b);...
        -old_v_eb_n(2) * tan(old_L_b) / (old_R_E + old_h_b)];
    omega_en_n = [v_eb_n(2) / (R_E + h_b);...
        -v_eb_n(1) / (R_N + h_b);...
        -v_eb_n(2) * tan(L_b) / (R_E + h_b)];
    
    % Obtain coordinate transformation matrix from the old attitude (w.r.t.
    % an inertial frame) to the new using (5.77)
    C_old_new = C_b_n' * (eye(3) - Skew_symmetric(omega_ie_n + 0.5 *... 
        omega_en_n + 0.5 * old_omega_en_n)  * tor_i) * old_C_b_n;

    % Calculate the approximate angular rate w.r.t. an inertial frame
    alpha_ib_b(1,1) = 0.5 * (C_old_new(2,3) - C_old_new(3,2));
    alpha_ib_b(2,1) = 0.5 * (C_old_new(3,1) - C_old_new(1,3));
    alpha_ib_b(3,1) = 0.5 * (C_old_new(1,2) - C_old_new(2,1));

    % Calculate and apply the scaling factor
    temp = acos(0.5 * (C_old_new(1,1) + C_old_new(2,2) + C_old_new(3,3)...
        - 1.0));
    if temp>2e-5 %scaling is 1 if temp is less than this
        alpha_ib_b = alpha_ib_b * temp/sin(temp);
    end %if temp
    
    % Calculate the angular rate
    omega_ib_b = alpha_ib_b / tor_i;
    
    % Calculate the specific force resolved about ECEF-frame axes
    % From (5.54),
    f_ib_n = ((v_eb_n - old_v_eb_n) / tor_i) - Gravity_NED(old_L_b,old_h_b)...
        + Skew_symmetric(old_omega_en_n + 2 * omega_ie_n) * old_v_eb_n;
    
    % Calculate the average body-to-NED coordinate transformation
    % matrix over the update interval using (5.84) and (5.86)
    mag_alpha = sqrt(alpha_ib_b' * alpha_ib_b);
    Alpha_ib_b = Skew_symmetric(alpha_ib_b);    
    if mag_alpha>1.E-8
        ave_C_b_n = old_C_b_n * (eye(3) + (1 - cos(mag_alpha)) /mag_alpha^2 ...
            * Alpha_ib_b + (1 - sin(mag_alpha) / mag_alpha) / mag_alpha^2 ...
            * Alpha_ib_b * Alpha_ib_b) -...
            0.5 * Skew_symmetric(old_omega_en_n + omega_ie_n) * old_C_b_n;
    else
        ave_C_b_n = old_C_b_n -...
            0.5 * Skew_symmetric(old_omega_en_n + omega_ie_n) * old_C_b_n;
    end %if mag_alpha
    
    % Transform specific force to body-frame resolving axes using (5.81)
    f_ib_b = inv(ave_C_b_n) * f_ib_n;
    
else
    % If time interval is zero, set angular rate and specific force to zero
    omega_ib_b = [0;0;0];
    f_ib_b = [0;0;0];
end %if tor_i
% Ends