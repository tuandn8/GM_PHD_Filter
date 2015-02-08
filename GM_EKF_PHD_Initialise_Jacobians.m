%GM_EKF_PHD_Initialise_Jacobians
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%In this file we calculate the Jacobians for the EKF in the PHD filter.
%These are for the motion and observation models, and the noise on each of
%these.

%Many of these are implemented as anonymous functions; if you are not familiar
%with these, see http://www.mathworks.com.au/help/matlab/matlab_prog/anonymous-functions.html

%% SENSOR (MEASUREMENT) (OBSERVATION) MODELLING
%We use the terms sensor model, measurement model and observation model
%interchangeably throughout the code.

%For a linear Kalman filter, the observation matrix is H.
%If we assume that we can directly observe all the dimensions of
%the state, H = eye(4) since the state X = [x; y; vx; vy]
%If we assume we can only observe position and not velocity, 
%H = [eye(2), zeros(2)]

%For a nonlinear extended Kalman filter:
% - the observation model is h. 
%   z = h(X)
%   h maps from the state space to the measurement space.
% - the inverse model is h^-1 or inv_h. 
%  X = inv_h(z)
%  inv_h maps from the measurement space to the state space.
% - the jacobian is h' or H
%  H = h' = dh/dX

%% Observation model h
%h converts a state to an expected observation
% z = h(X)
%It is a function of the target state AND sensor noise AND the sensor position.
%So the full function is z = h(X, epsilon, x_sensor)
%We only consider epsilon in Jacobian calculation, but we use x_sensor in he function h().
%We are using a range-bearing sensor z = [r; theta] for r in m, theta in radians
%State-space X = [x; y; vx; vy] but we neglect velocity for now.
%Bearing 0 is straight ahead of the sensor, positive is left, negative is right
%We will treat the sensor as being oriented to point along the global
%X-axis (i.e. sensor points to right in global frame).
%As in this diagram:
%          __O   <--target
%      ___/ |
% ___/      |delta_y          
%/    t     |   t is theta
%S--------->| 
% <-delta_x->
%^sensor is in the bottom left corner of the triangle
%Therefore the mapping is:
%r = (delta_x^2 + delta_y^2)^1/2 (Pythagoras' theorem, where delta_x and delta_y are the range from the sensor to the target in X and Y in local cartesian coordinates)
%theta = atan2(delta_y / delta_x) (trigonometry)

%xS and yS are the global coordinates of the sensor in X and Y
%hS is the heading of the sensor (relative to the global frame, 0 = to the right along the global X axis, positive left to pi, negative right to -pi).
%xL and yL are the global coordinates of the landmark in X and Y
%The returned range and bearing are in local sensor-centric coordinates
%(bearing 0 is directly in front of the sensor, positive is to the left)
h = @(xS, yS, hS, xL, yL) [hypot(xL - xS, yL - yS); atan2(yL - yS, xL - xS) - hS];%Subtract hS to convert to local coordinates, as the sensor would observe

%% Jacobian H of observation model calculations
%H = dh(x_k,0)/dx_k where x_k = m_k|k-1  for observation model z = h(x_k, eps_k) where x_k is
%target state and eps_k is the noise.
%There are three ways of calculating this:
%1. Numerically, using GM_EKF_PHD_Numerical_Jacobian.m
%2. Analytically, using the function Calculate_Jacobian_H.m
%3. Analytically, using an anonymous function that's identical to
%Calculate_Jacobian_H.m but has less documentation.
%To compare performance of these three, see Test_Jacobian_Calculation.m
%(the run-and-time feature is particularly useful to see how the numerical
%method is slower and the other two are equivalent).
%The advantage of the numerical method is that you can change the
%observation model without having to re-calculate the Jacobian manually
%(but you need to make sure that it takes in arguments the same way, or
%change GM_EKF_PHD_Numerical_Jacobian to take them in however you want).
%The disadvantage of the numerical method is that it is much slower (about
%7 times slower when I tested it).
%For now, I'll use the anonymous method as it takes up less space, but if you
%want to know how I derived it (it involves plugging equations into Wolfram
%Alpha) see Calculate_Jacobian_H.m
%If you want to change which one is used, uncomment/recomment as
%appropriate.
%Anonymous Jacobian calculation function:
calculate_Jacobian_H = @(xR,yR,xL,yL)[ [(xL - xR) / hypot(xL - xR,yL - yR) , (yL - yR) / hypot(xL - xR,yL - yR), 0, 0]; [ (yR - yL)/((xL - xR)^2 + (yL - yR)^2),  (xL - xR)/((xL - xR)^2 + (yL - yR)^2), 0, 0]; [0 0 1 0]; [0 0 0 1] ];
%Jacobian calculation function:
% calculate_Jacobian_H = @(xR,yR,xL,yL) Calculate_Jacobian_H(xR, yR, xL, yL);
%Numerical Jacobian function:
% calculate_Jacobian_H = @(xR,yR,xL,yL) GM_PHD_Numerical_Jacobian(h, [xR,yR,0],[xL,yL, 0, 0]);%h is the observation model, defined above. 

%% Inverse observation model inv_h
%inv_h converts an observation to an expected state
% X = inv_h(z)
% z = [r; theta] in metres and radians.
%r >= 0 and is univseral for local or global coordinates
% -pi < theta <= pi and is in local coordinates (local = sensor relative).
%Positive theta is CCW (pi/2 is left, 0 is up, -pi/2 is right)
%Even though state X is four-dimensional (x, y, vx, vy) we cannot infer
%velocity information from a single observation; it requires at least two.
%Therefore we will return a shortened state vector [x; y] from inv_h and
%pad it with zeros later.
%x right is positive, y up is positive, and both are in global coordinates.
%Zero X at theta = +-pi/2, positive X at -pi/2 < theta < pi/2 negative X at abs(theta) > pi/2
%x = r * cos(theta + pi/2)
%Zero Y at theta = 0 or pi, positive theta at 0 < theta < pi, negative,
%negative Y at theta < 0
%y = r * sin(theta + pi/2)
%r is range to target (rad), theta is bearing to target (rad), hS is bearing of
%sensor in global coordinates (rad)
inv_h = @(r, theta, xS, yS, hS)  [r .* cos(theta + hS) + xS; r .* sin(theta + hS) + yS];

%% Measurement covariance R calculation
%Measurement is 2x1 matrix
%Z = [r; theta]
%We augment this to include an implicit velocity observation, taken from a
%sequence of two consecutive sets of measurements, v = (newPos - oldPos)/dt
%for newPos and oldPos in global coordinates.
%Z = [r; theta; vx; vy];
%Including noise (the covariance of which is R)
%Z = [r + sigma_r; theta + sigma_theta; vx + sigma_vx; vy + sigma_vy]

%R = [ [sigma_r^2, 0]; [0, sigma_theta^2] ]; is the covariance of a range and bearing measurement in polar coordinates
%For sigma_r: For this simulation the value of sigma_r is kind of
%arbitrary. Let's just say 5m for simplicity.
sigma_r = 5; %Sensor range standard deviation in m
%For sigma_theta: Also kind of arbitrary, so we'll say 2 degrees
sigma_theta = deg2rad(2); %Sensor bearing standard deviation in radians 
%NOTE: These values are also used in GM_EKF_PHD_Simulate_Measurements to
%generate noise for measurements, so will have an effect on the simulation. This is most
%noticeable at long ranges from the sensor, where the error can grow quite
%large due to bearing error.
calculate_R_polar = @(sigma_r, sigma_theta) [[sigma_r^2, 0]; [0, sigma_theta^2]];

%For sigma_vx and sigma_vy:
%V = (inv_h(Z) - m_i(1:2))/dt
% V = f(X, dt) = [vx; vy] where f is a function that calculates velocity.
% X = [X1; X2]
% X1 = m_i = [x1; y1] 
% X2 = inv_h(z) = [x2; y2]
% z = [r; theta];
% dt = either measured from a sensor or a constant, we assume it has
% covariance of zero (our clock is perfectly accurate)
% V =  f(X, dt) = [ (x2 - x1)/dt; (y2 - y1)/dt]

%The covariance of the velocity is dependent on the covariance of m_i which
%is stored in P_i) and the covariance of Z (which is R)

% Jacobian_of_velocity_f = dV/dX = [dvx/dX; dvy/dX]
% Jacobian_of_velocity_f = [dvx/dx1, dvx/dx2 dvx/dy1 dvx/dy2]
%                [dvy/dx1, dvy/dx2 dvy/dy1 dvy/dy2]
% dvx/dX = [-1/dt 1/dt 0 0]
% dvy/dX = [0 0 -1/dt 1/dt]
% Jacobian_of_velocity_f = @(dt) [ [-1/dt 1/dt 0 0]; [0 0 -1/dt 1/dt]
Jacobian_of_velocity_f = @(dt) [ [-1/dt 1/dt 0 0]; [0 0 -1/dt 1/dt] ];

% Covariance of V = sigma_v = Jacobian_of_velocity_f * P_x * Jacobian_of_velocity_f' where
% P_x = [covariance_of_x1; covariance_of_x2];
% Covariance_of_x1 is stored in P (i.e. the uncertainty in target position)
% x2 = z = [r; theta] in polar coordinates
% Covariance_of_x2 = R, but it is in polar coordinates.

% To convert R to Cartesian coordinates
% z_c = inv_h(x2) = [r * cos(theta); r * sin(theta)];
% R_c = Jacobian_of_inv_h * R * Jacobian_of_h'
% Jacobian_of_inv_h = dinv_h/dx2 = dinv_h/dz since x2 = measurement vector
% dinv_h/dr = [cos(theta); sin(theta)];
% dinv_h/dtheta = [-rsin(theta); rcos(theta)];
% Jacobian_of_inv_h = [ [cos(theta), sin(theta)]; [-rsin(theta); rcos(theta)] ];
% Jacobian_of_inv_h = [ dinv_h_x/dr dinv_h_x/dtheta]
%                     [ dinv_h_y/dr dinv_h_y/dtheta]
%dinv_h_x / dr = cos(theta)
%dinv_h_x / dtheta = -r*sin(theta)
%dinv_h_y / dr = sin(theta)
%dinv_h_y / dtheta = r*cos(theta)
% Jacobian_of_inv_h = [ [cos(theta), -rsin(theta)]; [sin(theta), r*cos(theta)]];
calculate_Jacobian_of_inv_h = @(r, theta, rH) [ [cos(theta + rH), -r*sin(theta + rH)]; [sin(theta + rH), r*cos(theta + rH)] ];
% Therefore
% R_c = Jacobian_of_inv_h * R * Jacobian_of_inv_h';
% P_x = [P_x1; R_c];
% sigma_v = Jacobian_of_f * P_x * Jacobian_of_f'
calculate_R_cartesian = @(R, r, theta, rH) calculate_Jacobian_of_inv_h(r, theta, rH) * R * calculate_Jacobian_of_inv_h(r, theta, rH)';

%To calculate sigma_v, calculate R_cartesian and pass it through along with
%dt and the 2x2 covariance of the landmark.
calculate_R_velocity_cartesian = @(P_landmark, R_c, dt) Jacobian_of_velocity_f(dt) * [[P_landmark, zeros(2,2)]; [zeros(2,2), R_c]] * Jacobian_of_velocity_f(dt)'; %2x4 * 4x4 * 4x2 = 2x2 matrix [[sigma_vx^2, 0]; [0, sigma_vy^2]]

%% Jacobian U (Jacobian of measurement noise) calculation
%U = dh(m_k|k-1,eps_k)/deps_k where eps_k = 0 for observation model z = h(x_k, eps_k) where x_k is
%target state and eps_k is the noise.
%h = @(xR, yR, hR, xL, yL, vxL, vyL) [hypot(xL - xR, yL - yR) + eps_r; atan2(yL - yR, xL - xR) - pi/2 - hR + eps_theta; vxL + eps_vx; vyL + eps_vy];
%We assume that the noise is independent of the state; range error is not
%proportional to range, velocity noise is not proportional to velocity,
%etc.
%dh1/deps_r = 1
%dh1/deps_theta = 0
%dh1/deps_vx = 0
%dh1/deps_vy = 0
%dh2/deps_r = 0
%dh2/deps_theta = 1
%dh2/deps_vx = 0
%dh2/deps_vy = 0
%etc; we end up with an identity matrix
%Jacobian U is dh/d_epsilon = eye(4)
U = eye(4);

%% PREDICTION (MOTION) MODELLING
%% Jacobian F calculations
%We use the terms prediction model and motion model interchangeably
%throughout the code.

%For a linear Kalman filter, the prediction matrix is F_k-1
%For a nonlinear extended Kalman filter, the prediction function is phi and
%it takes the state as an argument.
%phi(m_k, dt) = phi([x; y; vx; vy], dt) = [x + vx * dt; y + vy * dt; vx; vy]
%This is for a point target moving with near-constant velocity, and neglects acceleration ax and ay, angular velocity omega, angular
%acceleration alpha (and probably a bunch of other terms).
%TODO: UPDATE. A constant velocity model is simple to implement but not
%ideal.
%We take a Jacobian F = d_phi / dx |x = x_k-1
%i.e. going across differentiate phi1 by x1, x2 ...
%going down differentiate phi1, phi2 ... by x1
%So row i is the gradient of the ith component function

%State X = [x; y; vx; vy]
%Prediction function phi(X,dt) = [x + vx * dt; y + vy * dt; vx; vy]
%We take a Jacobian F = d_phi / dx |x = x_k-1
%i.e. going across differentiate phi1 by x1, x2 ...
%going down differentiate phi1, phi2 ... by x1
%So row i is the gradient of the ith component function
%F = [ d(phi_x) / dx      d(phi_x) / dy     d(phi_x) / dvx    d(phi_x) / dvy   ]
%    [ d(phi_y) / dx      d(phi_y) / dy     d(phi_y) / dvx    d(phi_y) / dvy   ]
%    [ d(phi_vx) / dx     d(phi_vx) / dy    d(phi_vx) / dvx   d(phi_vx) / dvy  ]
%    [ d(phi_vy) / dx     d(phi_vy) / dy    d(phi_vy) / dvx   d(phi_vy) / dvy  ]
%F = [1 0 dt 0 ]
%    [0 1 0 dt ]
%    [0 0 1 0  ]
%    [0 0 0 1  ]
%So, Jacobian F of prediction model
calculate_Jacobian_F = @(dt) [ [1 0 dt 0]; [0 1 0 dt]; [0 0 1 0]; [0 0 0 1] ];

%% Jacobian G (Jacobian of prediction covariance) calculations
%The previous prediction function in the working for F was simplified to
%not include noise. Now we include noise (upsilon) in our calculations.
%Including noise, prediction function is:

%phi_x = x + upsilon_x + (vx + upsilon_vx) * dt   
%where estimated velocity is vx, true velocity is vx + upsilon_vx. 
%upsilon_x is some sort of disturbance in position unrelated to velocity.

%phi_y = y + upsilon_y + (vy + upsilon_vy) * dt 
%Estimated velocity is vy, true velocity is vy + upsilon_vy. 
%upsilon_y is some sort of disturbance in position unrelated to velocity.

%phi_vx = vx + upsilon_vx 
%Estimated velocity is vx, true velocity is vx + upsilon_vx where 
%upsilon_vx is some sort of disturbance in velocity; in our simplified
%constant-velocity model it's probably largely due to acceleration

%phi_vy = vy + upsilon_vy 
%Estimated velocity is vy, true velocity is vy + %upsilon_vy, where 
%upsilon_vy is some sort of disturbance in velocity; in our simplified
%constant-velocity model it's probably largely due to acceleration

%We need GQG' to be a 4x4 matrix. Therefore we will make G a 4x4 matrix
%Differentiating prediction model phi([x; y; vx; vy], dt) by the error 
%model [upsilon_x; upsilon_y; upsilon_vx; upsilon_vy]
%phi = [f_x; f_y; f_vx; f_vy]
%upsilon = [upsilon_x; upsilon_y; upsilon_vx; upsilon_vy]
%phi_x, phi_y, phi_vx and phi_vy including upsilon terms are defined above.

%G = d_phi / d_upsilon
%G = [dphi_x / dupsilon_x, dphi_x / dupsilon_y, dphi_x / dupsilon_vx, dphi_x / dupsilon_vy]
%    [dphi_y / dupsilon_x, dphi_y / dupsilon_y, dphi_y / dupsilon_vx, dphi_y / dupsilon_vy]
%    [dphi_vx / dupsilon_x, dphi_vx / dupsilon_y, dphi_vx / dupsilon_vx, dphi_vx / dupsilon_vy]
%    [dphi_vy / dupsilon_x, dphi_vy / dupsilon_y, dphi_vy / dupsilon_vx, dphi_vy / dupsilon_vy]
%G  = [1 0 dt 0]
%     [0 1 0 dt]
%     [0 0 1 0 ]
%     [0 0 0 1 ] 
calculate_Jacobian_G = @(dt) [[ 1, 0, dt, 0]; [0, 1, 0, dt]; [0, 0, 1, 0]; [0, 0, 0, 1] ];

%% Covariance matrix Q calculation
%Q is the covariance of the prediction. The one given by Vo & Ma for the
%linear problem doesn't seem to work with this EKF implementation; we end up with too many 
%false targets. I might look into this eventually and come up with a better calculation. 
%The values below are an effective hack and easy to modify if you want to.
upsilon_x = 1;
upsilon_y = 1;
upsilon_vx = 1;
upsilon_vy = 1;
Q = eye(4);
Q(1,1) = upsilon_x^2;
Q(2,2) = upsilon_y^2;
Q(3,3) = upsilon_vx^2;
Q(4,4) = upsilon_vy^2;