%GM_PHD_Estimate
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This file estimates the positions of targets tracked by the PHD filter.
%We need to extract the likely target positions from the PHD (i.e. we need to find the peaks of the PHD).
%This is actually fairly tricky. The naive approach is to pull out targets with the
%highest weights, but this is FAR from the best approach. A large covariance will pull down
%the peak size, and when targets are close together or have high covariances, there can be
%superposition effects which shift the peak.

%This just implements the method in Vo&Ma, which is pulling out every target with a weight over  
%weightThresholdToBeExtracted (defined in GM_PHD_Initialisation). There is
%the option of repeatedly printing out targets with rounded weights greater
%than 1 (i.e. if two or more strong targets are mergde and the weight
%rounds to 2/3/4/etc, display the target at that point multiple times when
%VERBOSE is set to 1). This will NOT change filter performance as the
%extracted state estimate is not fed back into the filter.
s = sprintf('Step 6: Estimate target states');
disp(s);
X_k = [];
X_k_P = [];
X_k_w = [];

%OUTPUT_MULTIPLE_HIGH_WEIGHT_TARGETS is set in GM_PHD_Initialisation
if(OUTPUT_MULTIPLE_HIGH_WEIGHT_TARGETS == 0)
    i = find(w_bar_k > weightThresholdToBeExtracted);
    X_k = m_bar_k(:,i);
    X_k_w = w_bar_k(:,i);
    for j = 1:length(i)
        thisI = i(j);
        P_range = calculateDataRange4(thisI);

        thisP = P_bar_k(:,P_range);
        X_k_P = [X_k_P, thisP];
    end
else
    %If a target has a rounded weight greater than 1, output it multiple
    %times. VERBOSE must be set to 1 to see the effects of this.
    for i = 1:size(w_bar_k,2)
       for j = 1:round(w_bar_k(i))
            X_k = [X_k, m_bar_k(:,i)];
            X_k_w = [X_k_w, w_bar_k(i)];
            P_range = calculateDataRange4(i);
            thisP = P_bar_k(:,P_range);
            X_k_P = [X_k_P, thisP];
       end
    end
end

if(VERBOSE == 1)
    s = sprintf('\t%d targets beleived to be valid:', size(X_k, 2));
    disp(s);
    for i = 1:size(X_k, 2)
        P_range = calculateDataRange4(i);
       s = sprintf('\t\tTarget %d at %3.4f %3.4f, P %3.4f %3.4f, W %3.5f', i, X_k(1, i), X_k(2,i), X_k_P(1, P_range(1)), X_k_P(2, P_range(2)), X_k_w(i));
       disp(s);
    end
end

%Store history for plotting.
X_k_history = [X_k_history, X_k];