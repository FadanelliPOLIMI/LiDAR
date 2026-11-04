function [ptCloudClean, metrics] = denoiseAdaptive(ptCloudInput, config)
    % DENOISEADAPTIVE - Adaptive denoising using tail-to-median ratio 
    % and Statistical Outlier Removal (SOR). Returns a pointCloud object.
    % Dual-Mode Support: Laptop (High Fidelity) vs. Edge / Jetson Nano (Optimized)
    
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    
    if ~isa(ptCloudInput, 'pointCloud') || ptCloudInput.Count == 0
        error('PipelineError:InvalidInput', 'step2a_denoiseAdaptive expects a non-empty pointCloud object.');
    end
    
    initialCount = ptCloudInput.Count;
    
    % --- Statistical Outlier Removal (SOR) ---
    tic;
    fprintf('  -> Applying Statistical Outlier Removal (Neighbors: %d, Threshold: %.1f)...\n', ...
        config.sorNeighbors, config.sorThreshold);
    
    % pcdenoise natively returns a pointCloud object and handles attribute preservation
    [ptCloudClean, ~, outlierIndices] = pcdenoise(ptCloudInput, ...
        'NumNeighbors', config.sorNeighbors, ...
        'Threshold', config.sorThreshold);
    
    elapsedTime = toc;
    finalCount = ptCloudClean.Count;
    retentionRate = (finalCount / initialCount) * 100;
    
    fprintf('  -> [SOR] Neighbors: %d, Threshold: %.1f | Removed: %d pts | Retained: %.1f%% (%.2fs)\n', ...
        config.sorNeighbors, config.sorThreshold, length(outlierIndices), retentionRate, elapsedTime);
    fprintf('  -> Denoising complete in %.2f seconds.\n', elapsedTime);
    fprintf('--------------------------------------------\n');
    
    % --- 3. Populate Metrics Output ---
    metrics = struct();
    metrics.initialCount  = initialCount;
    metrics.finalCount    = finalCount;
    metrics.removedCount  = length(outlierIndices);
    metrics.retentionRate = retentionRate;
    metrics.elapsedTime   = elapsedTime;
    metrics.status        = 'success';
end