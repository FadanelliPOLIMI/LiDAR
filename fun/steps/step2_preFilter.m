function [ptCloudClean, config, stage2Metrics] = step2_preFilter(ptCloudRaw, config)
    % STEP2_PREPROCESSINGPIPELINE - Unified Stage 2 Preprocessing Pipeline
      
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end
    
    initialCount = ptCloudRaw.Count;
    if initialCount == 0
        error('PipelineError:EmptyInput', 'The input raw point cloud for Stage 2 is empty.');
    end
    
    ticTotal = tic;
    
    % =====================================================================
    % STEP 2A: Adaptive Denoising + Safety Feedback Loop
    % =====================================================================
    fprintf('\n>>> Executing Step 2A: Adaptive Denoising...\n');
    try
        ptCloudClean2A = ptCloudRaw;
        metrics2A = struct();
        minRetention2A = 0.80; 
        
        thresholdMultipliers = [1.0, 1.5, 2.0];
        isResolved2A = false;
        
        for attempt = 1:length(thresholdMultipliers)
            config_try = config;
            config_try.sorThreshold = config.sorThreshold * thresholdMultipliers(attempt);
            
            [ptCloudClean2A, tempMetrics] = denoiseAdaptive(ptCloudRaw, config_try);
            retentionRatio = ptCloudClean2A.Count / initialCount;
            
            fprintf('  -> Attempt %d (Multiplier %.1fx): Retains %.1f%% of points...\n', ...
                attempt, thresholdMultipliers(attempt), retentionRatio * 100);
                
            if retentionRatio >= minRetention2A
                isResolved2A = true;
                metrics2A = tempMetrics;
                metrics2A.status = sprintf('success_multiplier_%.1f', thresholdMultipliers(attempt));
                break;
            end
        end
        
        if ~isResolved2A
            warning('PipelineWarning:OverDenoisingMax', 'All SOR relaxations failed retention floor. Bypassing denoising to protect data.');
            ptCloudClean2A = ptCloudRaw;
            metrics2A.status = 'bypassed_over_denoising';
        end
        
    catch ME
        warning('Step2A execution failed (%s). Falling back to raw input.', ME.message);
        ptCloudClean2A = ptCloudRaw;
        metrics2A = struct('status', 'fallback_raw');
    end
    
    % =====================================================================
    % STEP 2B: Adaptive Voxel Quality Gate + Iterative Voxel Adjustment
    % =====================================================================
    fprintf('\n>>> Executing Step 2B: Adaptive Voxel Quality Gate (Iterative Loop)...\n');
    try
        baseVoxel = config.voxelSize;
        voxelScaleFactors = [1.0, 0.75, 0.5]; 
        isResolved2B = false;
        
        for attempt = 1:length(voxelScaleFactors)
            config_try = config;
            config_try.voxelSize = baseVoxel * voxelScaleFactors(attempt);
            
            [ptCloudClean2B, tempMetrics] = voxelCheckAdaptive(ptCloudClean2A, config_try);
            
            if strcmpi(tempMetrics.status, 'voxel_accepted') || strcmpi(tempMetrics.status, 'sparse_passthrough')
                isResolved2B = true;
                metrics2B = tempMetrics;
                metrics2B.status = sprintf('success_voxel_%.3f', config_try.voxelSize);
                
                % LOCK VOXEL SIZE IN CONFIG
                config.voxelSize = config_try.voxelSize;
                fprintf('  -> [CONFIG UPDATED] Pipeline voxelSize locked to: %.3f m\n', config.voxelSize);
                break;
            end
        end
        
        if ~isResolved2B
            warning('PipelineWarning:VoxelGateFailed', 'All scales failed. Retaining dense cloud. Reverting to base voxel for downstream config.');
            ptCloudClean2B = ptCloudClean2A;
            metrics2B = struct('status', 'fallback_dense_passthrough');
        end
        
    catch ME
        warning('Step2B execution failed (%s). Falling back to Step 2A output.', ME.message);
        ptCloudClean2B = ptCloudClean2A;
        metrics2B = struct('status', 'fallback_2a');
    end
    
    % =====================================================================
    % STEP 2C: PCA Coarse Geometric Pre-Filter
    % =====================================================================
    fprintf('\n>>> Executing Step 2C: PCA Coarse Pre-Filter...\n');
    try
        if ptCloudClean2B.Count == 0, error('PipelineError:EmptyInput', 'Input to Step 2C is empty.'); end
        
        initialCount2C = ptCloudClean2B.Count;
        
        % Compute normals once and cache directly on the pointCloud object
        normals = pcnormals(ptCloudClean2B, config.kNeighbors);
        ptCloudClean2B.Normal = normals;
        
        verticality = abs(normals(:,3));
        verticality(isnan(verticality)) = 0;
        
        baseThreshold = config.verticalityThreshold;
        allowedThresholds = [baseThreshold, max(0.2, baseThreshold - 0.10), max(0.1, baseThreshold - 0.20)];
        minAcceptableRetention2C = 0.70;
        
        isResolved2C = false;
        bestMask = [];
        metrics2C = struct();
        
        for attempt = 1:length(allowedThresholds)
            testThreshold = allowedThresholds(attempt);
            isGroundCandidate = (verticality >= testThreshold);
            retentionRatio = sum(isGroundCandidate) / initialCount2C;
            
            fprintf('  -> Attempt %d: Testing verticality threshold %.2f (Retains %.1f%%)...\n', ...
                attempt, testThreshold, retentionRatio * 100);
                
            if retentionRatio >= minAcceptableRetention2C
                bestMask = isGroundCandidate;
                isResolved2C = true;
                metrics2C.status = sprintf('success_threshold_%.2f', testThreshold);
                break;
            end
        end
        
        if isResolved2C
            ptCloudClean2C = select(ptCloudClean2B, bestMask);
            metrics2C.retentionRate = (ptCloudClean2C.Count / initialCount2C) * 100;
        else
            warning('PipelineWarning:SlopeFailureMax', 'All dynamic verticality thresholds failed retention floor. Bypassing pre-filter.');
            ptCloudClean2C = ptCloudClean2B;
            metrics2C.status = 'bypassed_slope_protection_max';
        end
        
    catch ME
        warning('Step2C execution failed (%s). Falling back to Step 2B output.', ME.message);
        ptCloudClean2C = ptCloudClean2B;
        metrics2C = struct('status', 'fallback_2b');
    end
    
    % Final assignment
    ptCloudClean = ptCloudClean2C;
    totalElapsedTime = toc(ticTotal);

    % --- STAGE 2 VISUALIZATION ---
    if isfield(config, 'visible') && config.visible
        fprintf('  -> [VISUALIZATION] Launching side-by-side comparison...\n');
        visualizeDenoising(ptCloudRaw, ptCloudClean);
    end
    
    stage2Metrics = struct('initialCount', initialCount, 'finalCount', ptCloudClean.Count, ...
        'retentionRate', (ptCloudClean.Count / initialCount) * 100, ...
        'totalExecutionTime', totalElapsedTime, ...
        'metrics2A', metrics2A, 'metrics2B', metrics2B, 'metrics2C', metrics2C);
    
    fprintf('\n--------------------------------------------\n');
    fprintf('STAGE 2 AUTONOMOUS PIPELINE COMPLETE\n');
    fprintf('  -> Initial Points : %d\n', initialCount);
    fprintf('  -> Retained Points: %d (%.1f%% of original)\n', ptCloudClean.Count, stage2Metrics.retentionRate);
    fprintf('  -> Final Voxel Size: %.3f m\n', config.voxelSize);
    fprintf('  -> Total Time     : %.2f seconds\n', totalElapsedTime);
    fprintf('--------------------------------------------\n');
end