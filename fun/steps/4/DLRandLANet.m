function [ptCloudRoad] = DLRandLANet(ptCloudFiltered, config)
    % DLRANDLANET - Deep Learning RandLA-Net Road Segmentation with Spatial Tiling
    % Main entry point utilizing a corrected local helper function for batched inference.
    
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end
    
    fprintf('\n[DL RANDLA-NET] Starting Batched 3D Deep Learning Segmentation...\n');
    tic;
    
    try
        % --- 1. MODEL LOADING ---
        fprintf('  -> Loading RandLA-Net segmenter ("dales")...\n');
        segmenter = randlanet("dales");
        
        % --- 2. SELECT BLOCK SIZE BASED ON HARDWARE ---
        if strcmpi(config.mode, 'edge')
            blockSize = [30, 30]; % 30m x 30m tiles for Jetson Nano RAM safety
            fprintf('  -> [EDGE MODE] Using 30x30m spatial tiles for batched inference.\n');
        else
            blockSize = [80, 80]; % 80m x 80m tiles for laptop performance
            fprintf('  -> [LAPTOP MODE] Using 80x80m spatial tiles for batched inference.\n');
        end
        
        % --- 3. EXECUTE BATCHED INFERENCE VIA HELPER FUNCTION ---
        allLabels = runBatchedInference(segmenter, ptCloudFiltered, blockSize);
        
        % --- 4. EXTRACT ROAD MASK ---
        if iscategorical(allLabels)
            groundMask = (allLabels == "Ground") | (allLabels == "Terrain") | (allLabels == "Road");
            vegMask    = (allLabels == "Vegetation") | (allLabels == "Tree") | (allLabels == "Bush");
            if ~any(groundMask), groundMask = true(size(allLabels)); end
            if ~any(vegMask), vegMask = false(size(allLabels)); end
        else
            groundMask = (allLabels == 1);
            vegMask    = (allLabels == 2);
        end
        
        pureRoadMask = groundMask & ~vegMask;
        groundIndices = find(pureRoadMask);
        
        if isempty(groundIndices)
            error('No road points identified after batched DL classification.');
        end
        
        ptCloudRawDL = select(ptCloudFiltered, groundIndices);
        
        % --- 5. CONDITIONAL VISUALIZATION (HEADLESS SAFE) ---
        if isfield(config, 'visible') && config.visible && ~strcmpi(config.mode, 'edge')
            fprintf('  -> [VISUALIZATION] Rendering Batched DL Classification Results...\n');
            locsFull = single(ptCloudFiltered.Location);
            colors = zeros(ptCloudFiltered.Count, 3, 'single');
            colors(pureRoadMask, :) = repmat(single([0.2, 0.7, 0.3]), sum(pureRoadMask), 1);
            
            ptCloudColored = pointCloud(locsFull, 'Color', colors);
            figure('Name', 'Batched DL Classification 3D', 'Color', 'w', 'Position', [50, 50, 1000, 700]);
            pcshow(ptCloudColored);
            title('Batched DL Classification: Green = Road');
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            drawnow;
        end
        
        % --- 6. POST-PROCESSING CLEANUP ---
        fprintf('  -> Removing stray points and isolated clusters...\n');
        ptCloudRoad = pcdenoise(ptCloudRawDL, 'NumNeighbors', 20, 'Threshold', 1.0);
        
        isolateCluster = true;
        if isfield(config, 'isolateMainCluster'), isolateCluster = config.isolateMainCluster; end
        
        if isolateCluster && ptCloudRoad.Count > 1000
            [labelsCluster, numClusters] = pcsegdist(ptCloudRoad, single(0.5));
            if numClusters > 1
                clusterSizes = histcounts(labelsCluster, 1:(numClusters+1));
                [~, largestClusterIdx] = max(clusterSizes);
                ptCloudRoad = select(ptCloudRoad, labelsCluster == largestClusterIdx);
                fprintf('     * Spurious clusters removed. Retaining main road body (%d points).\n', ptCloudRoad.Count);
            end
        end
        
        fprintf('  -> Batched DL filtering complete!\n');
        
    catch ME
        % --- SMRF FALLBACK ---
        warning('Batched Deep Learning failed (%s). Activating SMRF geometric fallback.', ME.message);
        try
            [~, ~, ptCloudRoad] = segmentGroundSMRF(ptCloudFiltered, ...
                'MaxWindowRadius', 5, 'ElevationThreshold', 0.2, 'SlopeThreshold', 0.2);
        catch
            ptCloudRoad = ptCloudFiltered;
        end
    end
    
    elapsedTime = toc;
    fprintf('  -> DL Module Total Time: %.2f seconds. Final road points: %d\n', elapsedTime, ptCloudRoad.Count);
end

% =========================================================================
% LOCAL HELPER: CORRECTED SPATIAL TILE BATCHING
% =========================================================================
function allLabels = runBatchedInference(segmenter, ptCloudInput, blockSize)
    bpc = blockedPointCloud(ptCloudInput, blockSize);
    sizeBlocks = bpc.SizeInBlocks; % Correct property for dimensions in blocks
    
    fprintf('  -> Point cloud partitioned into blocks (Grid: %dx%dx%d).\n', ...
        sizeBlocks(1), sizeBlocks(2), sizeBlocks(3));
    
    allLabels = [];
    
    for i = 1:sizeBlocks(1)
        for j = 1:sizeBlocks(2)
            for k = 1:sizeBlocks(3)
                try
                    ptCloudBlock = getBlock(bpc, [i, j, k]); % Correct retrieval function
                catch
                    continue; 
                end
                
                if isempty(ptCloudBlock) || ptCloudBlock.Count < 10, continue; end
                
                % Downsample block if excessively dense
                if ptCloudBlock.Count > 50000
                    ptCloudBlockWork = pcdownsample(ptCloudBlock, 'gridNearest', 0.2);
                else
                    ptCloudBlockWork = ptCloudBlock;
                end
                
                % Run inference on block
                labelsBlock = segmentObjects(segmenter, ptCloudBlockWork);
                
                % Reproject labels via k-NN if downsampled
                if ptCloudBlockWork.Count < ptCloudBlock.Count
                    locsFull = single(ptCloudBlock.Location);
                    locsDown = single(ptCloudBlockWork.Location);
                    kdtreeDown = KDTreeSearcher(locsDown);
                    [idxK, ~] = knnsearch(kdtreeDown, locsFull, 'K', 1);
                    labelsFullBlock = labelsBlock(idxK);
                else
                    labelsFullBlock = labelsBlock;
                end
                
                % Accumulate labels
                allLabels = [allLabels; labelsFullBlock]; %#ok<AGROW>
            end
        end
    end
    
    if isempty(allLabels)
        error('Batched DL inference returned empty labels across all blocks.');
    end
end