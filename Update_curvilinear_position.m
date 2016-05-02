function [L_b,lambda_b,h_b]= Update_curvilinear_position(tor_i,old_L_b,...
    old_lambda_b,old_h_b,old_v_eb_n,v_eb_n)
%Update_curvilinear_position - Updates latitude, longitude, and height by
%integrating the velocity
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
%   v_eb_n        current velocity of body w.r.t earth, resolved about 
%                 north, east, and down (m/s)%
% Outputs:
%   L_b       current latitude (rad)
%   lambda_b  current longitude (rad)
%   h_b       current height (m)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Calculate meridian and transverse radii of curvature
[old_R_N,old_R_E]= Radii_of_curvature(old_L_b);

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

% Ends