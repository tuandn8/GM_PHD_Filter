%% Calculate_Jacobian_H
%Calculates the Jacobian of a range-bearing measurement model
%xS and yS are the position of the sensor
%xL, yL, vxL, vyL are the state of the landmark (position and velocities)
%Velocities are not used.
%This function is re-implemented as a one-line anonymous function in
%GM_EKF_PHD_Initialise_Jacobians but I kept this here in case you want more
%info on how it was calculated, or want to change it.
function H = Calculate_Jacobian_H(xS, yS, xL, yL, vxL, vyL)
%H = [d(f_r)/dX; d(f_theta)/dX ]
%H = [d(f_r) / dx         d(f_r) / dy       d(f_r) / dvx      d(f_r) / dvy]
%    [d(f_theta) / dx     d(f_theta) / dy   d(f_theta)/ dvx   d(f_theta) / dvy]
%f_r = (delta_x^2 + delta_y^2)^1/2  = ((xL - xR)^2 + (yL - yR)^2)^1/2
%f_theta = atan(delta_y/delta_x)
%d(f_r) / dx = (xL - xR) / range (where range = hypot(xL - xR, yL - yR))
%d(f_r) / dy = (yL - yR) / range
%d(f_theta) / dx = (yR - yL) / range^2
%d(f_theta) / dy = (xL - xR) / range^2

% We use wolfram alpha to differentiate things.
% "Derivative of ((x - a)^2 + (y - b)^2)^1/2 with respect to x" gives
% (x - a) / ((x - a)^2 + (y - b)^2)^1/2
% "Derivative of ((x - a)^2 + (y - b)^2)^1/2 with respect to y" gives
% (y - b) / ((x - a)^2 + (y - b)^2)^1/2
%"Derivative of atan2((y-a),(x-b)) with respect to x" gives
%(a - y) / ((x - a)^2 + (y - b)^2) <--------- NOTE that this is (a - y) on
%numerator, not (y - a)
%"Derivative of atan2((y-a),(x-b)) with respect to y" gives
%(x - b) / ((x - a)^2 + (y - b)^2)

range = hypot(xL - xS, yL - yS);
delta_x = xL - xS;
delta_y = yL - yS;

dfr_dx = delta_x / range;
dfr_dy = delta_y / range;
dfr_dvx = 0;
dfr_dvy = 0;

dftheta_dx = -delta_y / range^2;
dftheta_dy = delta_x / range^2;
dftheta_dvx = 0;
dftheta_dvy = 0;

dfvx_dx = 0;
dfvx_dy = 0;
dfvx_dvx = 1;
dfvx_dvy = 0;

dfvy_dx = 0;
dfvy_dy = 0;
dfvy_dvx = 0;
dfvy_dvy = 1;

H = [ [dfr_dx, dfr_dy, dfr_dvx, dfr_dvy]; [dftheta_dx, dftheta_dy, dftheta_dvx, dftheta_dvy]; [dfvx_dx, dfvx_dy, dfvx_dvx, dfvx_dvy ]; [dfvy_dx, dfvy_dy, dfvy_dvx, dfvy_dvy] ];

end