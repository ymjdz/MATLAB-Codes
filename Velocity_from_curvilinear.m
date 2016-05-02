function v_eb_n = Velocity_from_curvilinear(tor_i,old_L_b,...
    old_lambda_b,old_h_b,old_v_eb_n,L_b,lambda_b,h_b)
%Velocity_from_curvilinear - updates velocity by differentiating
%latitude, longitude, and height
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 31/3/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   old_L_b       previous latitude (rad)
%   old_lambda_b  previous longitude (rad)
%   old_h_b       previous height (m)
%   old_v_eb_n    previous velocity of body w.r.t earth, resolved about 
%                 north, east, and down (m/s)
%   L_b           current latitude (rad)
%   lambda_b      current longitude (rad)
%   h_b           current height (m)
% Outputs:
%   v_eb_n        current velocity of body w.r.t earth, resolved about 
%                 north, east, and down (m/s)%

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Calculate meridian and transverse radii of curvature
[old_R_N,old_R_E]= Radii_of_curvature(old_L_b);
[R_N,R_E]= Radii_of_curvature(L_b);

% Differentiate latitude, longitude, and height
lat_rate = (L_b - old_L_b) / tor_i;
long_rate = (lambda_b - old_lambda_b) / tor_i;
ht_rate = (h_b - old_h_b) / tor_i;

% Derive the current velocity using (5.56)
v_eb_n(1,1) = (old_R_N + h_b) * (2 * lat_rate - old_v_eb_n(1) / (old_R_N +...
    old_h_b));
v_eb_n(2,1) = ((R_E + h_b) * cos(L_b)) * (2 * long_rate - old_v_eb_n(2) /...
    ((old_R_E + old_h_b) * cos(old_L_b)));
v_eb_n(3,1) = -2 * ht_rate - old_v_eb_n(3);

% Ends