%GM_PHD_Filter
%Version 1.10, last modified 7th January 2014
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 
%With:
%- some Kalman filter update code by Tim Bailey, taken from his website http://www-personal.acfr.usyd.edu.au/tbailey/software/
%- error_ellipse by AJ Johnson, taken from Matlab Central http://www.mathworks.com.au/matlabcentral/fileexchange/4705-errorellipse

%The GM-PHD algorithm is from Ba-Ngu Vo & Wing-Kin Ma in:
%B.-N. Vo, W.-K. Ma, "The Gaussian Mixture Probability Hypothesis Density
%Filter", IEEE Transactions on Signal Processing, Vol 54, No. 11, November 2006, pp4091-4104

%I have implemented both the linear Kalman filter and extended Kalman
%filter versions of this algorithm; switch between these by
%setting/clearing the variable USE_EKF in GM_PHD_Initialisation.

%Ba-Ngu Vo has kindly allowed me to include his implementation of the Optimal Subpattern Assignment
%(OSPA) metric proposed by D. Schuhmacher, Ba-Tuong Vo & Ba-Ngu Vo in
% Schuhmacher, D.; Ba-Tuong Vo; Ba-Ngu Vo, "A Consistent Metric for Performance Evaluation of Multi-Object Filters," Signal Processing, IEEE Transactions on , vol.56, no.8, pp.3447,3457, Aug. 2008
%This uses
%- ospa_dist by Ba-Ngu Vo, taken from http://ba-ngu.vo-au.com/vo/OSPA_for_Tracks.zip
%- Hungarian by Alex Melin (to whom I am also much obliged), also taken from http://ba-ngu.vo-au.com/vo/OSPA_for_Tracks.zip

%The OSPA metric is not essential for the functioning of the filter but
%provides a nice way of analysing performance.

%See the README.txt, the comments and the Vo & Ma paper for more
%information about what this code actually does.

clear all;
close all;
clc;

%Step 0: Initialisation
%The EKF version must initialise the Jacobian functions used to linearise
%the observation/prediction models. The simulated measurements are also very
%different; the linear KF is direct observations of target state, the EKF is
%range-bearing measurements. The observation and prediction covariances are
%also different.
GM_PHD_Initialisation;
if USE_EKF == 0
    GM_PHD_Simulate_Initialise;
else
    GM_EKF_PHD_Simulate_Initialise;
    GM_EKF_PHD_Initialise_Jacobians;
end

%In Vo&Ma, the targets are known at filter initialisation.
%If we want to know about them, set KNOWN_TARGET to 1 in GM_PHD_Initialisation.
%Otherwise they should be initialised after being detected a few times.
%HOWEVER this is not guaranteed - sometimes due to noise or missed
%detections, one or both of the targets will not be tracked. This is just 
%part of the filter and the birth_intensity function.
if KNOWN_TARGET == 1
    t1start = [simTarget1Start(1:2); simTarget1Vel];
    t2start = [simTarget2Start(1:2); simTarget2Vel];
    m_birth = [t1start, t2start];
    w_birth = [birth_intensity(t1start), birth_intensity(t2start)];
    P_birth = [covariance_birth, covariance_birth];
    numBirthedTargets = 2;
end

%Main loop
while (k < endTime)%k = timestep
    k = k + 1;
    s = sprintf('======ITERATION %d======', k);
    disp(s);
        
    %Step Sim: Generate sensor Measurements
    %If you want to use this code with your own data or for a different problem,
    %replace this function with your own.
    if USE_EKF == 0
        GM_PHD_Simulate_Measurements;  %Linear KF measurements are simulated direct observations [X; Y] of the target positions
    else
        GM_EKF_PHD_Simulate_Measurements; %EKF measurements are simuated range-bearing [r; theta] of the target position from a fixed sensor.
    end
    
    %Linear KF use fixed matrices for prediction and update.
    %EKF calculates Jacobians to linearise the prediction and observation
    %models. There is need for an inverse sensor model to map from sensor
    %to target space for some calculations.
    if(USE_EKF == 0)
        %Step 1: Prediction for birthed/spawned targets 
        GM_PHD_Predict_Birth; 
        %Step 2: Prediction for existing targets
        GM_PHD_Predict_Existing;
        %Step 3: Construction of PHD update components
        GM_PHD_Construct_Update_Components;
        %Step 4: Update targets with measurements
        GM_PHD_Update;
    else
        %Step 1: Prediction for birthed/spawned targets 
        GM_EKF_PHD_Predict_Birth; %EKF prediction uses Jacobians to linearise the prediction model and predict the targets forward in time
        %Step 2: Prediction for existing targets
        GM_EKF_PHD_Predict_Existing;
        %Step 3: Construction of PHD update components
        GM_EKF_PHD_Construct_Update_Components;
        %Step 4: Update targets with measurements
        GM_EKF_PHD_Update;
    end
    
    %Step 5: Prune targets
    GM_PHD_Prune;
    %Step 6: Estimate position of targets
    GM_PHD_Estimate

    %Step 7: Create birthed-targets-list to add next iteration in Step 1.
    %Not a formal part of Vo&Ma but an essential step!
    %The EKF version uses an inverse sensor model.
    if(USE_EKF == 0)
        GM_PHD_Create_Birth; 
    else
        GM_EKF_PHD_Create_Birth;
    end
    
    %Step Metric: Calculate performance metric
    GM_PHD_Calculate_Performance_Metric;
    
    %Step Plot: Generate graphs
    %The EKF version uses an inverse sensor model.
    if USE_EKF == 0
        GM_PHD_Simulate_Plot;
    else
       GM_EKF_PHD_Simulate_Plot;
    end
    
    if(VERBOSE == true)
        pause;%Pause to allow reading of the text
    end

end

