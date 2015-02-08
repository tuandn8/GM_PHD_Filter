%GM_PHD_Calculate_Performance_Metric
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This uses the Optimal Subpattern Assignment (OSPA) metric proposed by 
%Schuhmacher, D.; Ba-Tuong Vo; Ba-Ngu Vo, "A Consistent Metric for Performance Evaluation of Multi-Object Filters," Signal Processing, IEEE Transactions on , vol.56, no.8, pp.3447,3457, Aug. 2008
%cutoff_c and order_p are tuning parameters set in GM_PHD_Initialisation
if(CALCULATE_OSPA_METRIC == 1)
    X = X_k;
    Y = [simTarget1History(:,k), simTarget2History(:,k)];
    if(k >= simTarget3SpawnTime)
        Y = [Y, simTarget3History(:,k-simTarget3SpawnTime+1)];
    end

    metric = ospa_dist(X, Y, cutoff_c, order_p);%Use Ba-Ngu Vo's implementation
    metric_history = [metric_history, metric];
    
end