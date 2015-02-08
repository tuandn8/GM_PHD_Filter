%CalculatePerformanceMetric
%Last modified 21 November 2013
%Matlab code by Bryan Clarke b.clarke@acfr.usyd.edu.au 

%This is an implementation of the Optimal Subpattern Assignment
%(OSPA) metric proposed by Schuhmacher et al in 
%Schuhmacher, D.; Ba-Tuong Vo; Ba-Ngu Vo, "A Consistent Metric for Performance Evaluation of Multi-Object Filters," Signal Processing, IEEE Transactions on , vol.56, no.8, pp.3447,3457, Aug. 2008
%X is the estimated state in the form [ [x1; y1; vx1; vy1], [x2; y2; vx2;
%vy2] , ...]
%Y is the ground truth in the form [ [x1; y1; vx1; vy1], [x2; y2; vx2;
%vy2] , ...]
%This isn't actually important, as we will swap the labels so that X
%is the label of the shorter vector and Y is the label of the longer.
%cutoff_c and order_p are parameters that control the metric calculation;
%cutoff is a saturation threshold, order controls how punishing it is to
%larger errors versus smaller ones. See the paper by Schuhmacher et al to
%get a handle on these in more detail.
%NOTE: This implementation is still a work in progress and is a bit buggy. Use with caution.
%NOTE: We only use 2D OSPA, for position; we don't use the velocities.
function ospa = CalculateOSPAMetric(X, Y, cutoff_c, order_p)

    m = size(X, 2);%Length of vector X
    n = size(Y, 2);%Length of vector Y

    alphas = cutoff_c * ones(1, n);%Initialise to cutoff, overwrite if there is a shorter value
    bestOMATCost = -1;
    bestOMATDataAssoc_i = [];
    
    %m (i.e. the length of X) needs to be less than or equal to n (the length of Y)
    if(m > n)%Swap them if this is not the case. X and Y are just labels that can be applied to either vector; whichever one is estimate or truth is not important.
        tmpX = X;
        tmpm = m;
        X = Y;
        m = n;
        Y = tmpX;
        n = tmpm;
    end
    if(m > 0)%If there are other values, we need to find the best data association.
        %We calculate all potential combinations (ie. subsampling without
        %replacement)
        comboSize = m;
        valuesSampled = 1:n;
        allCombos = combnk(valuesSampled, comboSize);
        nCombos = size(allCombos, 1);
        
        %Then for each combination, we calculate every permutation (i.e.
        %different ways to order the numbers) to find all possible data
        %associations
        for i = 1:nCombos
            thisCombo = allCombos(i,:);%The combination we are using
            allDataAssocs = perms(thisCombo);
            nDataAssocs = size(allDataAssocs, 1);
            %We check all the data associations for this combination
            for j = 1:nDataAssocs
                thisDataAssoc = allDataAssocs(j,:);%An ordered list of the indices of Y to match them against the values in X                
                thisY = Y(:,thisDataAssoc);
                thisOMATCost = CalculateOMATMetric(X, thisY, order_p);

                if(bestOMATCost < 0) || (thisOMATCost < bestOMATCost) %If this is the first iteration, or the best so far, save
                    bestOMATCost = thisOMATCost;
                    bestOMATDataAssoc_i = thisDataAssoc;
                end
            end
        end
        
        %Now that we have the best data association, we use it to calculate
        %alphas for the matched points
        for i = 1:m
           thisX = X(:,i);
           thisY = Y(:,bestOMATDataAssoc_i(i));%Use the data association array to pull out the corresponding value in Y
           alphas(i) = min(cutoff_c, norm(thisX - thisY) ^ order_p);%New alpha is either the distance or the cutoff, whichever is lower.                
        end
        %Add cutoff for unmatched targets
        for i = 1:(n - m)
            alphas(m+i) = cutoff_c;
        end
        
    end
    alphasSum = sum(alphas) / n;
    ospa = alphasSum ^ (1/order_p);
end

%We assume that the length of X = the length of Y, since we only use this
%for finding the best data association for the OSPA metric.
%X and Y are 2d arrays [[x1; y1], [x2; y2] ...[xN; yN]]
%order_p is an integer greater than zero
function omat = CalculateOMATMetric(X, Y, order_p)
    m = size(X, 2);
    omat = 0;
    for i = 1:m
        thisX = X(:,i);
        thisY = Y(:,i);
        dx = thisX(1) - thisY(1);
        dy = thisX(2) - thisY(2);
        omat = omat + hypot(dx, dy) ^ order_p;
    end
    omat = omat / m;
    omat = omat ^ (1/order_p);%Since length of X = length of Y, the OMAT distance simplifies to this
end