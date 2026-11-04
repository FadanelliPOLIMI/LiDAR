function cost = costFunctionHST(params, ptCloudRaw)
    % COSTFUNCTIONHST - Cost function for Bayesian optimization of heuristic parameters.
    % Evaluates point retention ratio and ground planarity (MAD Z).
    
    try
        [ptCloudGround, ~, ~, ~] = heuristicSelfTrain(...
            ptCloudRaw, ...             % 1. Raw point cloud
            'min', ...                  % 2. methodRef
            params.binSize, ...         % 3. binSize
            params.macroSize, ...       % 4. macroSize
            'road', ...                 % 5. profileMode
            params.minSearchHeight, ... % 6. minSearchHeight
            params.K1, ...              % 7. K1
            params.K2, ...              % 8. K2
            params.maxNumSplits, ...    % 9. maxNumSplits
            params.greenThreshold, ...  % 10. greenThreshold
            false);                     % 11. enablePlot (OFF for bayesopt speed)
    
    catch ME
        % --- ROBUST ERROR DIAGNOSTICS ---
        errLine = 0;
        if ~isempty(ME.stack), errLine = ME.stack(1).line; end
        fprintf('\n[BAYESOPT ERROR CAUGHT]: %s (Line %d)\n', ME.message, errLine);
        cost = 50.0; % High penalty cost for failed evaluation
        return;
    end
    
    numGround = ptCloudGround.Count;
    numTotal  = ptCloudRaw.Count;
    
    % --- RETENTION & PENALTY BARRIERS ---
    if numGround < 100
        cost = 40.0; 
        return;
    end
    
    ratio = numGround / numTotal;
    
    targetRatio = 0.45;
    centerPenalty = 5.0 * (ratio - targetRatio)^2;
    lowerBarrier = 10.0 * log(1 + exp(-20 * (ratio - 0.15)));
    upperBarrier = 10.0 * log(1 + exp( 20 * (ratio - 0.90)));
    ratioCost = centerPenalty + lowerBarrier + upperBarrier;
    
    % --- PLANARITY COST (SINGLE PRECISION SAFE) ---
    xyzGround = single(ptCloudGround.Location);
    zMAD = median(abs(xyzGround(:,3) - median(xyzGround(:,3))));
    planarCost = min(10.0, zMAD / 0.04);
    
    cost = ratioCost + planarCost;
    
    if isnan(cost) || isinf(cost)
        cost = 30.0;
    end
end