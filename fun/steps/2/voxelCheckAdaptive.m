function [ptCloudFinal, metrics] = voxelCheckAdaptive(inputCloud, config)
    % VOXELCHECKADAPTIVE - Advanced geometric quality control (Spacing + RMSE + MaxError)
    % Accepts a pointCloud object and configuration, returning a processed pointCloud 
    % and a metrics structure compatible with the Stage 2 orchestration pipeline.
    % Dual-Mode Support: Laptop (High Fidelity) vs. Edge / Jetson Nano (Optimized)
    
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    
    if ~isa(inputCloud, 'pointCloud')
        error('PipelineError:InvalidInput', 'voxelCheckAdaptive expects a pointCloud object.');
    end
    ptCloudAOI = inputCloud;
    
    % Intelligent Spacing Control
    X_orig = double(ptCloudAOI.Location(:,1));
    Y_orig = double(ptCloudAOI.Location(:,2));
    aoiArea = max(1e-6, (max(X_orig) - min(X_orig)) * (max(Y_orig) - min(Y_orig)));
    avgSpacing = 1 / sqrt(ptCloudAOI.Count / aoiArea);

    if avgSpacing >= config.voxelSize
        fprintf('  -> [ADAPTIVE LOGIC] SPARSE CLOUD DETECTED (Spacing %.3fm >= Voxel %.3fm). Propagating full-res.\n', ...
            avgSpacing, config.voxelSize);
        ptCloudFinal = ptCloudAOI; 
        rmseVal = 0.0; maxErrorVal = 0.0; meanBias = 0.0;
        status = 'sparse_passthrough';

    else
        % Secure Downsampling
        ptCloudDown = pcdownsample(ptCloudAOI, 'gridAverage', config.voxelSize);

        % OPTIMIZED RMSE CALCULATION (Using K-D Trees instead of grids)
        [~, dists] = findNearestNeighbors(ptCloudDown, ptCloudAOI.Location, 1);
        meanBias    = mean(dists);
        maxErrorVal = max(dists);
        rmseVal     = sqrt(mean(dists.^2));

        fprintf('  -> [VOXEL CHECK] RMSE: %.1f mm | Max Error: %.1f mm | Bias: %.1f mm\n', ...
            rmseVal * 1000, maxErrorVal * 1000, meanBias * 1000);

        % Adaptive Decision
        if (rmseVal <= config.rmseThreshold) && (maxErrorVal <= config.maxErrorThreshold)
            fprintf('-> Voxel accepted: homogeneous surface.\n');
            ptCloudFinal = ptCloudDown; 
            status = 'voxel_accepted';
        else
            fprintf('-> High local variation detected. Retaining dense point cloud.\n');
            ptCloudFinal = ptCloudAOI; 
            status = 'fallback_dense';
        end
    end

    metrics = struct('rmseVal', rmseVal, 'maxErrorVal', maxErrorVal, ...
        'meanBias', meanBias, 'status', status, 'avgSpacing', avgSpacing);
end