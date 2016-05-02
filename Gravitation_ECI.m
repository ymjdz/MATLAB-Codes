function gamma = Gravitation_ECI(r_ib_i)
%Gravitation_ECI - Calculates gravitational acceleration resolved about 
%ECI-frame axes
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.

% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   r_ib_i  Cartesian position of body frame w.r.t. ECI frame, resolved
%           about ECI-frame axes (m)
% Outputs:
%   gamma   Acceleration due to gravitational force (m/s^2)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

%Parameters
R_0 = 6378137; %WGS84 Equatorial radius in meters
mu = 3.986004418E14; %WGS84 Earth gravitational constant (m^3 s^-2)
J_2 = 1.082627E-3; %WGS84 Earth's second gravitational constant

% Begins

% Calculate distance from center of the Earth
mag_r = sqrt(r_ib_i' * r_ib_i);

% If the input position is 0,0,0, produce a dummy output
if mag_r==0
    gamma = [0;0;0];
    
% Calculate gravitational acceleration using (2.141)
else
    z_scale = 5 * (r_ib_i(3) / mag_r)^2;
    gamma = -mu / mag_r^3 *(r_ib_i + 1.5 * J_2 * (R_0 / mag_r)^2 *...
        [(1 - z_scale) * r_ib_i(1); (1 - z_scale) * r_ib_i(2);...
        (3 - z_scale) * r_ib_i(3)]);

end % if

% Ends