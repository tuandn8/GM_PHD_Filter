%% Test_Jacobian_Calculation
%Last modified 3rd September 2013

%Tests observation Jacobians, and contains calculations for manual
%calculation of observation Jacobian. To see performance differences
%between numerical and analytical implementations, use Run and Time. They
%should all produce the same output, but GM_PHD_Numerical_Jacobian should
%run slower.

% The observation Jacobian is the derivative of the observation function with respect to the state
% Each matrix cell is the partial derivative of one observation function by one state variable.
% Each matrix row is one observation function.
% Each matrix column is one state variable.
% Each matrix row is the gradient of one observation function along a certain axis.
% If state is X and observation function is F,
% Jacobian H = dF/dX
% If F = [f_r; f_theta]
% and X = [xL; yL; vxL; vyL] or just [x; y; vx; vy]
% and the sensor state is xR, yR, hR
% H1 = [df_r/dx, df_r/dy df_r/dvx df_r/dvy]
% H2 = [df_theta/dx, df_theta/dy df_theta/dvx df_theta/dvy]

%f_r = ((xL - xR)^2 + (yL - yR)^2) ^ 1/2
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

% When the target is straight ahead/to side and moving towards/away, there is zero change in bearing and high change in range
% H1 = [1
% And derivatives of anything with respect to vx and vy will be 0.
% 
% What values do we expact?
% When sensor is pointing along the X-axis,
% - any movement on the X-axis should have maximum change in range and minimum change in bearing
% - any movement on the Y-axis should have less change in range than X (due to triangle inequality) and maximum change in bearing
% - any change in vx or vy should have no change
% So when radar is 0,0,0 and target is [1, 0]
% H1 = [high low 0 0]
%H2 = [low high 0 0]

GM_EKF_PHD_Initialise_Jacobians;%Initialise observation model h
%Initialise sensor position
xS = 0;%X position in m. X is positive right
yS = 0;%Y position in m. Y is positive up
hS = 0;%Heading in radians. 0 is to the right, positive is left.
%Initialise landmark position. Ideally, don't put this as the exact same
%position as the sensor
xL = 1.5;%X position in m
yL = 2.8;%Y position in m
vxL = 0;%X velocity in m/s
vyL = 0;%Y velocity in m/s

x_sensor = [xS, yS, hS];
x_landmark = [xL, yL, vxL, vyL];

%Anonymous function
calculate_Jacobian_H = @(xR,yR,xL,yL)[ [(xL - xR) / hypot(xL - xR,yL - yR) , (yL - yR) / hypot(xL - xR,yL - yR), 0, 0]; [ (yR - yL)/((xL - xR)^2 + (yL - yR)^2),  (xL - xR)/((xL - xR)^2 + (yL - yR)^2), 0, 0]; [0 0 1 0]; [0 0 0 1] ];

J1 = Calculate_Jacobian_H(xS, yS, xL, yL);%Analytical function
J2 = calculate_Jacobian_H(xS,yS,xL,yL);%Anonymous analytical function
J3 = GM_EKF_PHD_Numerical_Jacobian(h, x_sensor, x_landmark);%Numerical function
J1
J2
J3



