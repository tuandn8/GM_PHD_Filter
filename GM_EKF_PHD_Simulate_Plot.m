%GM_EKF_PHD_Simulate_Plot
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 
%error_ellipse is by AJ Johnson, taken from Matlab Central http://www.mathworks.com.au/matlabcentral/fileexchange/4705-errorellipse

%This file plots the current measurements, true target position and estimated target
%position, as well as a history of estimated target positions.

%Measurements

figure(1)
%Plot all measurements, including clutter, as black 'x'
if(~isempty(Z))
    Z_xy = inv_h(Z(1,:), Z(2,:), zeros(size(Z(1,:))) + x_sensor(1), zeros(size(Z(1,:))) + x_sensor(2), zeros(size(Z(1,:))));%Add x_sensor(1) and x_sensor(2) to convert to global coordinates for plotting
    plot(Z_xy(1,:), Z_xy(2,:) , 'xk');
end

%Plot noisy measurements of true target position(s), as black 'o'
if(~isempty(zTrue))
    Z_xy_true = inv_h(zTrue(1,:), zTrue(2,:), zeros(size(zTrue(1,:))) + x_sensor(1), zeros(size(zTrue(1,:))) + x_sensor(2), zeros(size(zTrue(1,:))));%Add x_sensor(1) and x_sensor(2) to convert to global coordinates for plotting
    plot(Z_xy_true(1,:), Z_xy_true(2,:) , 'ok');   
end

%Plot target 1 true position as red dots     
plot(simTarget1History(1,:), simTarget1History(2,:), '.-r');  
%Plot target 2 true position as blue dots  
plot(simTarget2History(1,:), simTarget2History(2,:), '.-b');  
%Plot target 3 true position as green dots 
if(~isempty(simTarget3History)) 
    plot(simTarget3History(1,:), simTarget3History(2,:), '.-g');  
end
%Plot tracked targets as magenta circles
if(~isempty(X_k_history))
    plot(X_k_history(1,:), X_k_history(2,:), 'om');
end

xlabel('X position');
ylabel('Y position');
title('Simulated targets and measurements');
axis square;

%For extracted targets, plot latest target as cyan triangle, and draw an
%error ellipse to show uncertaintyty
if(~isempty(X_k))
    plot(X_k(1,:), X_k(2,:), '^c');
    [nRows, nCols] = size(X_k);
    for c = 1:nCols
       thisMu = X_k(1:2, c);
       covRange = calculateDataRange4(c);
       thisCov = X_k_P(:,covRange);
       thisCov = thisCov(1:2, 1:2); %We only care about position
       error_ellipse(thisCov, thisMu);
    end
    if(DRAW_VELOCITIES == 1)
          %Draw velocities of targets   
          quiver(X_k(1,:), X_k(2,:), X_k(3,:), X_k(4,:))          
    end
end

%Measurements
figure(2);
hold on;
subplot(2,1,1);
if(~isempty(Z))
    plot(k, Z_xy(1,:), 'xk');%X coord of clutter measurements
end
if(~isempty(zTrue))
    plot(k, Z_xy_true(1,:), 'ok');%X coord of true measurement
end
if(~isempty(X_k))
    plot(k, X_k(1,:), 'om');%Estimated target
end
subplot(2,1,2);
hold on;
if(~isempty(Z))
    plot(k, Z_xy(2,:), 'xk');%Y coord of clutter measurements
end
if(~isempty(zTrue))
    plot(k, Z_xy_true(2,:), 'ok');%Y coord of true measurement
end
if(~isempty(X_k))
    plot(k, X_k(2,:), 'om');%Estimated target
end

%OSPA error metric plot
if(CALCULATE_OSPA_METRIC == 1)
    figure(3);
    clf;
    hold on;
    plot(metric_history, 'x-b');
    axis([0 100 0 cutoff_c]);
    xlabel('Simulation step');
    ylabel('OSPA error metric (higher is worse)');
    title('OSPA error metric for this test');
end
