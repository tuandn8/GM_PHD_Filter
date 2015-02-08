%% GM_EKF_PHD_Construct_Update_Components
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file creates the components needed for performing an extended Kalman filter update on the
%targets using the measurement.
s = sprintf('Step 3: Constructing update components for all targets, new and existing.');
disp(s);

%We need to clear the data structures each iteration
eta = [];
S = [];
K = [];
P_k_k = [];

xR = x_sensor(1);
yR = x_sensor(2);
hR = x_sensor(3);
R_polar = calculate_R_polar(sigma_r, sigma_theta);%Sensor covariance, polar

for j = 1:numTargets_Jk_k_minus_1
    
    m_j = mk_k_minus_1(:,j);
    eta_j = zeros(4,1);%Expected observation
    eta_j(1:2) = h(xR,yR,hR,m_j(1), m_j(2));%Observation model.
    eta_j(3:4) = [m_j(3); m_j(4)];%Assume constant velocity.
    P_range = calculateDataRange4(j); %4x4 array

    H = calculate_Jacobian_H(xR, yR, m_j(1), m_j(2));

    PHt = Pk_k_minus_1(:,P_range) * H'; %Taken from Tim Bailey's EKF code. 4x2 array
    
    r = eta_j(1);
    theta = eta_j(2);
    %We need to extend the sensor covariance R to include the covariance of
    %the speed estimation. See GM_PHD_Initialise_Jacobians for details on
    %this calculation.
    R_cartesian = calculate_R_cartesian(R_polar, r, theta, hR);%sensor covariance, cartesian
    R_velocity = calculate_R_velocity_cartesian(Pk_k_minus_1(1:2,P_range(1:2)), R_cartesian, dt);%We need the landmark position uncertainty, the sensor uncertainty (in cartesian space) and the time delta
    R = [ [R_polar, zeros(2,2)]; [zeros(2,2), R_velocity] ];
    
    %Calculate K via Tim Bailey's method.
    S_j = U * R * U' + H * PHt;    %U is set in GM_PHD_Initialise_Jacobians.
    
    %At this point, Tim Bailey's code makes S_j symmetric. In this case, it leads to the matrix being non-positive definite a lot of the time and chol crashes.
    %So we won't do that. 
    SChol= chol(S_j);

    SCholInv =  SChol \ eye(size(SChol)); % triangular matrix, invert via left division
    W1 = PHt * SCholInv;
    
    HPHt = H * PHt;

    K_j = W1 * SCholInv';
    
    P_j = Pk_k_minus_1(:,P_range) - W1*W1';%4x4 array
    %End Tim Bailey's code.

    eta = [eta, eta_j];
    S = [S, S_j];
    K = [K, K_j];
    P_k_k = [P_k_k, P_j]; 
end