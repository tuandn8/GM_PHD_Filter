%GM_EKF_PHD_Simulate_Initialise
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%We implement the simulation described in example 1 of Vo&Ma 2006, the one
%designed for the linear KF PHD filter. Although a second, different
%example is listed for the EKF version, I haven't gotten around to
%implementing it yet.

%Most of this is unchanged from the example used in the linear KF PHD
%filter. The big changes for the EKF are in
%GM_EKF_PHD_Initialise_Jacobians.

%% Control parameters
x_sensor = [0; 0; 0];%The position of the simulated range-bearing sensor. Heading 0 is along the positive X-axis. Counter clockwise is positive, clockwise is negative, bearings are bounded -pi < theta <= pi
noiseScaler = 1.0;%Adjust the strength of the noise on the measurements by adjusting this
nClutter = 50; %Assume constant 50 clutter measurements. Since clutter is Poisson distrbuted it might be more accurate to use nClutter = poissrnd(50) if you have the required Matlab toolbox. Constant 50 clutter works well enough for simulation purposes.

%% Target parameters
endTime = 100;%Duration of main loop.
simTarget1Start = birth_mean1;
simTarget2Start = birth_mean2;

simTarget1End = [500, -900]';
simTarget2End = [900, -500]';
simTarget3End = [-200, -750]';

simTarget1Vel = (simTarget1End(1:2) - simTarget1Start(1:2)) / endTime;
simTarget2Vel = (simTarget2End(1:2) - simTarget2Start(1:2)) / endTime;
simTarget3Vel = [-20; -4];
simTarget1Start(3:4) = simTarget1Vel;
simTarget2Start(3:4) = simTarget2Vel;
simTarget3Start(3:4) = simTarget3Vel;

simTarget1History = simTarget1Start;
simTarget2History = simTarget2Start;
simTarget3History = [];

simTrueMeasurementHistory = [];
simNoisyMeasurementHistory = {};%We use a cell array so that we can have rows of varying length.
simVehicleStateHistory = {};%We store a history of vehicle positions at the same time as measurements are made.

simTarget1State = simTarget1Start;
simTarget2State = simTarget2Start;
simTarget3State = [];

simTarget3SpawnTime = 66;%Target 3 is spawned from target 1 at t = 66s

%Set up for plot
%Measurements and targets plot
figure(1);
clf;
hold on;
axis([-1000 1000 -1000 1000]);
xlim([-1000 1000]);
ylim([-1000 1000]);

%X and Y measurements plot
figure(2);
clf;
subplot(2,1,1);
hold on;
axis([0, 100, -1000, 1000]);
xlabel('Simulation step');    
ylabel('X position of measurement (m)');
title('Measurement X coordinates');
subplot(2,1,2);
hold on;
axis([0, 100, -1000, 1000]);
xlabel('Simulation step');    
ylabel('Y position of measurement (m)');
title('Measurement Y coordinates');

%Performance metric plot
if(CALCULATE_OSPA_METRIC == 1)
    figure(3);
    clf;
    xlabel('Simulation step');
    ylabel('OSPA error metric (higher is worse)');
end