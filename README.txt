GM_PHD_Filter
Version 1.09, 13th December 2013

Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au with:
- some Kalman filter update code by Tim Bailey, taken from his website http://www-personal.acfr.usyd.edu.au/tbailey/software/
- error_ellipse by AJ Johnson, taken from Matlab Central http://www.mathworks.com.au/matlabcentral/fileexchange/4705-errorellipse
Algorithm by Ba-Ngu Vo & Wing-Kin Ma in:
B.-N. Vo, W.-K. Ma, "The Gaussian Mixture Probability Hypothesis Density Filter", IEEE Transactions on Signal Processing, Vol 54, No. 11, November 2006, pp4091-4104

This is an implementation of a gaussian mixture probability hypothesis density filter (GM-PHD) for a simulated tracking problem. The problem specification is given in the above paper - in summary, two targets move through the environment, there is a lot of clutter on the measurement, and about halfway through a third target spawns off one of the two targets.

I made a few changes, either because I couldn't understand how Vo&Ma did it, or because I wanted to make it closer to my target problem. I extended the measurement vector to include both target position AND velocity (the filter they describe tracks position and velocity but only observes position). Velocity is observed as just being dx/dt, change in position over time, between this new observation and the previous position of this target. Targets are either birthed or spawned depending on which initialisation weight function would give a higher weight; they are given the appropriate initialisation covariance, but apart from this there is no other difference in instantiation. Vo & Ma's state estimation involves repeatedly outputting targets that have a rounded weight greater than 1; I have this off by default but it can be turned on by setting OUTPUT_MULTIPLE_HIGH_WEIGHT_TARGETS and VERBOSE to 1 in GM_PHD_Initialisation.

I've also implemented the Extended Kalman Filter version specified by Vo & Ma, but not the EKF simulation. To switch from linear KF to EKF, set USE_KF in GM_PHD_Initialisation to 1. Everything runs out of the same file, GM_PHD_Filter. The EKF uses either totally separate or totally identical files to the linear version. I considered intermingling the EKF code into the KF using a bunch of IF statements to make maintenance easier, but I was worried this would come at the cost of readability. Therefore the only place you will see IF(USE_EKF == 1) is in GM_PHD_Filter.

I've also included Ba-Ngu Vo's implementation of the Optimal Subpattern Assignment (OSPA) metric proposed by Schuhmacher et al in 
Schuhmacher, D.; Ba-Tuong Vo; Ba-Ngu Vo, "A Consistent Metric for Performance Evaluation of Multi-Object Filters," Signal Processing, IEEE Transactions on , vol.56, no.8, pp.3447,3457, Aug. 2008
This uses
- ospa_dist by Ba-Ngu Vo, taken from http://ba-ngu.vo-au.com/vo/OSPA_for_Tracks.zip
- Hungarian by Alex Melin, also taken from http://ba-ngu.vo-au.com/vo/OSPA_for_Tracks.zip

The files are mostly implemented as scripts. The main file is GM_PHD_Filter. The structure is pretty straightforward, as it mostly just calls the other scripts in a big loop.

For the linear Kalman filter, there are two files that initialise everything - GM_PHD_Initialise and GM_PHD_Simulate_Initialise. For the extended Kalman filter, there is GM_PHD_Initialisation, GM_EKF_PHD_Simulate_Initialise, and GM_EKF_PHD_Initialise_Jacobains. Changing values in these will change how the filter runs.

The other files are pretty easy to follow if you have a look in GM_PHD_Filter.m

There isn't much output on the console by default, most of it is in the graphs. Have a read of GM_PHD_Simulate_Plot to understand and edit what the different coloured dots mean. In summary, black X's are measurements, black X's with a circle around them are measurements corresponding to true targets, red/green/blue dots are true target positions, magenta circles are tracked targets, cyan triangles are the targets at the current filter iteration. If you set DRAW_VELOCITIES to 1 in GM_PHD_Initialisation, velocity arrows will be drawn on. The arrow shows direction, the length indicates magnitude (long arrow = high speed).

To see more filter output, set VERBOSE to 1 in GM_PHD_Initialisation. I can't remember exactly what this prints out, but it's more information about the inner working of the filter, and you can use it to add your own output statements wherever you want. I cut most of mine out because they were cluttering up the code.

To see the performance metric, set CALCULATE_OSPA_METRIC to 1 in GM_PHD_Initialisation. It appears as an extra line graph. High values on the line graph = high error value.

See license.txt for licensing information.

See ReleaseNotes.txt for changes between versions.

===A few remarks on the Kalman filter vs the extended Kalman filter===
If you've gotten this far I'm going to assume you understand what a KF and EKF are, so I am going to focus on the specifics of my implementation. The extended Kalman filter uses Jacobians to linearise the nonlinear prediction/observation models. These Jacobians can be calculated analytically (by hand or using a computer program; I use Wolfram Alpha as it's very easy to use) or numerically (very easy to code up but it runs much slower). I've included code for both but commented out the ones not being used. You can play around with or replace them as you see fit; see GM_EKF_PHD_Initialise_Jacobians and Test_Jacobian_Calculation.

The nonlinear observation model h() is used to map from the state space [x; y; vx; vy] to the measurement space [r; theta], primarily to generate expected observations of a target at a certain position. We also need to have an inverse observation inv_h() to perform the reverse, mapping from the measurement space [r; theta] to the state space [x; y; vx; vy] (but since we don't observe the velocity, it doesn't output velocity information and this has to be added later or not at all). The observation model is used in GM_EKF_PHD_Simulate_Measurements and GM_EKF_PHD_Construct_Update_Components. The inverse observation model is used in GM_EKF_PHD_Update, GM_EKF_PHD_Simulate_Plot and GM_PHD_Create_Birth.

If you want to replace these with different observation models, you're going to have to change these occurrences as well as the respective Jacobians.

I haven't implemented the EKF simulation given in Vo & Ma; both the EKF and KF run on the KF simulation. The prediction covariance Q given for this works well for the linear KF but not for the EKF. I set Q to eye(4) and it works; this value could probably use some further investigation and tweaking but this code is only a simulation, and a real application will probably have more grounded information on what values to set here.

Just in case someone doesn't want to use the EKF version, I'm including a zip file containing the linear-Kalman-filter-only version of the GM-PHD (version 1.05).

-- Bryan Clarke, b.clarke@acfr.usyd.edu.au