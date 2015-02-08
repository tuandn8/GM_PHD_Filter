%GM_PHD_Simulate_Measurements
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file generates simulated measurement data for the linear KF simulation
%described in Vo&Ma. There is a nonlinear simulation described in the paper
%but I have not gotten around to implementing it.
%The sensor measurements are range-bearing measurements, similar to those
%from an FMCW radar. The expected error magnitude is independent of
%measurement (i.e. range error is not proportional to range, bearing error
%is not proportional to bearing). 

%There will be gaussian noise on the measurement and Poisson-distributed clutter
%in the environment. 
%Note: It is possible to get no measurements if the target is not detected
%and there is no clutter

%If you want to use this PHD filter implementation for another problem, you
%will need to replace this script with another one that populates Z,
%zTrue, and simMeasurementHistory (Z is used in a lot of the update code,
%zTrue and simMeasurementHistory are used in GM_PHD_Simulate_Plot, and 
%simMeasurementHistory is used in GM_EKF_PHD_Create_Birth)

%There will be gaussian noise on the measurement and Poisson clutter
%in the environment. 
%Note: It is possible to get no measurements if the target is not detected
%and there is no clutter
s = sprintf('Step Sim: Simulating measurements.');
disp(s);

%Simulate target movement
F = calculate_Jacobian_F(dt);
simTarget1State = F * simTarget1State;
simTarget2State = F * simTarget2State;
if(~isempty(simTarget3State))
    simTarget3State = F * simTarget3State;
end
%Spawn target 3 when k = 66
if(k == simTarget3SpawnTime)
    simTarget3State = simTarget1State;
    simTarget3State(3:4) = simTarget3Vel;
end

%Save target movement
simTarget1History = [simTarget1History, simTarget1State];
simTarget2History = [simTarget2History, simTarget2State];
simTarget3History = [simTarget3History, simTarget3State];

%First, we generate some clutter in the environment.
clutter = zeros(2,nClutter);%The observations are of the form [x; y]
for i = 1:nClutter
    clutterX = rand * (xrange(2) - xrange(1)) + xrange(1); %Random number between xrange(1) and xrange(2), uniformly distributed.
    clutterY = rand * (yrange(2) - yrange(1)) + yrange(1); %Random number between xrange(1) and xrange(2), uniformly distributed.
    
    clutterRTheta = h(x_sensor(1), x_sensor(2), x_sensor(3), clutterX, clutterY);
    
    clutter(:,i) = clutterRTheta;
end

%We are not guaranteed to detect the target - there is only a probability
%that we will, controlled by prob_detection
detect1 = rand;
detect2 = rand; 
detect3 = rand;
%Target 1
if(detect1 > prob_detection)
    measR1 = [];
    measTheta1 = [];
else
    simMeas1 = h(x_sensor(1), x_sensor(2), x_sensor(3), simTarget1State(1), simTarget1State(2));%Generate measurement
    measR1 = simMeas1(1) + sigma_r * randn * noiseScaler;%Add gaussian noise. We could add the noise at the measurement-generating stage, by adding it to the simulated target state.
    measTheta1 = simMeas1(2) + sigma_theta * randn * noiseScaler;%Add gaussian noise. We could add the noise at the measurement-generating stage, by adding it to the simulated target state.
end
%Target 2
if(detect2 > prob_detection)
    measR2 = [];
    measTheta2 = [];
else
    simMeas2 = h(x_sensor(1), x_sensor(2), x_sensor(3), simTarget2State(1), simTarget2State(2));
    measR2 =  simMeas2(1) + sigma_r * randn * noiseScaler;
    measTheta2 = simMeas2(2) + sigma_theta * randn * noiseScaler;
end
%Target 3
if(k >= simTarget3SpawnTime) && (detect3 <= prob_detection)
    simMeas3 = h(x_sensor(1), x_sensor(2), x_sensor(3), simTarget3State(1), simTarget3State(2)); 
    measR3 = simMeas3(1) + sigma_r * randn * noiseScaler;
    measTheta3 = simMeas3(2) + sigma_theta * randn * noiseScaler;
else
    measR3 = [];
    measTheta3 = [];
end

%Generate true measurement
Z = [ [measR1 measR2 measR3]; [measTheta1 measTheta2 measTheta3] ];
zTrue = Z;

%Append clutter
Z = [Z, clutter];
simMeasurementHistory{k} =  Z;