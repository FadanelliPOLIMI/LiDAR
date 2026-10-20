function cost = costFunctionAT(params, ptCloudRaw)
% EVALUATEGROUNDFILTERCOSTSMOOTH - Versione con diagnostica errori

try
    [ptCloudGround, ~, ~, ~] = step3c_autoTunerFilter(...
        ptCloudRaw, ...             % 1. Nuvola grezza
        'min', ...                  % 2. methodRef
        params.binSize, ...         % 3. binSize
        params.macroSize, ...       % 4. macroSize (es. 25.0)
        'road', ...                 % 5. profileMode
        params.minSearchHeight, ... % 6. minSearchHeight (ora incluso!)
        params.K1, ...              % 7. K1
        params.K2, ...              % 8. K2
        params.maxNumSplits, ...    % 9. maxNumSplits
        params.greenThreshold, ...  % 10. greenThreshold
        false);                     % 11. enablePlot (OFF per velocità in bayesopt)
catch ME
    % --- DIAGNOSTICA: Stampa l'errore esatto nella console di MATLAB ---
    fprintf('\n[ERRORE INTERNO CAUGHT]: %s (Linea %d)\n', ME.message, ME.stack(1).line);
    cost = 50.0; 
    return;
end

numGround = ptCloudGround.Count;
numTotal  = ptCloudRaw.Count;

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

xyzGround = double(ptCloudGround.Location);
zMAD = median(abs(xyzGround(:,3) - median(xyzGround(:,3))));
planarCost = min(10.0, zMAD / 0.04);

cost = ratioCost + planarCost;

if isnan(cost) || isinf(cost)
    cost = 30.0;
end
end