function [ptCloudRoadOnly] = extractGeometricRoad(ptCloudInput, config, isPcaFailed)
    % EXTRACTGEOMETRICROAD - SMRF Ground Teacher & Main Cluster Isolation with Plotting
    % BEST FOR 1ST ROAD IMAGE
    % if ~isfield(config, 'xmax'), config.xmax = 110; end
    % if ~isfield(config, 'xmin'), config.xmin = 7.5; end
    % if ~isfield(config, 'ymax'), config.ymax = 10.1; end
    % if ~isfield(config, 'ymin'), config.ymin = 4.9; end

    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    if nargin < 3 || isempty(isPcaFailed)
        isPcaFailed = isfield(config, 'geoTransform') && isfield(config.geoTransform, 'pcaFailed') && config.geoTransform.pcaFailed;
    end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end

    % --- 1. CONFIGURATION DEFAULTS ---
    if ~isfield(config, 'maxIntensity'),       config.maxIntensity       = single(10000); end
    if ~isfield(config, 'maxWindowRadius'),    config.maxWindowRadius    = 5; end
    if ~isfield(config, 'elevationThreshold'), config.elevationThreshold = single(0.09); end
    if ~isfield(config, 'slopeThreshold'),     config.slopeThreshold     = single(0.20); end
    if ~isfield(config, 'isolateMainCluster'), config.isolateMainCluster = true; end

    % --- GEOMETRY-AWARE ADAPTATION (Using isPcaFailed) ---
    if isPcaFailed
        fprintf('  -> [GEOMETRY] PCA Unaligned: Relaxing slope threshold for isotropic robustness.\n');
        config.slopeThreshold = config.slopeThreshold * 1.5; % More forgiving on diagonal slopes
    else
        fprintf('  -> [GEOMETRY] PCA Aligned: Using strict directional slope thresholds.\n');
    end

    ptCloudWork = ptCloudInput;

    % --- 2. INTENSITY FILTERING ---
    if isprop(ptCloudWork, 'Intensity') && ~isempty(ptCloudWork.Intensity)
        intensityMask = (ptCloudWork.Intensity <= config.maxIntensity);
        if ~all(intensityMask)
            ptCloudWork = select(ptCloudWork, intensityMask);
            fprintf('  -> Intensity filter applied (max: %.0f): Retained %d / %d pts.\n', ...
                config.maxIntensity, ptCloudWork.Count, ptCloudInput.Count);
        end
    end

    if ptCloudWork.Count == 0
        warning('PipelineWarning:EmptyCloud', 'Point cloud empty after intensity filtering. Returning input.');
        ptCloudRoadOnly = ptCloudInput;
        return;
    end

    % --- 3. SMRF GROUND SEGMENTATION ---
    fprintf('  -> Running SMRF Ground Teacher (Window: %d, ElevThresh: %.3f, SlopeThresh: %.3f)...\n', ...
        config.maxWindowRadius, config.elevationThreshold, config.slopeThreshold);
    try
        [~, ~, groundPtCloud] = segmentGroundSMRF(ptCloudWork, ...
            'MaxWindowRadius', config.maxWindowRadius, ...
            'ElevationThreshold', config.elevationThreshold, ...
            'SlopeThreshold', config.slopeThreshold);
    catch ME
        warning('SMRF execution failed (%s). Falling back to default parameters.', ME.message);
        [~, ~, groundPtCloud] = segmentGroundSMRF(ptCloudWork);
    end

    if groundPtCloud.Count == 0
        ptCloudRoadOnly = ptCloudWork;
        return;
    end

    % --- 4. CONTIGUOUS MAIN ROAD CLUSTER ISOLATION ---
    if config.isolateMainCluster
        if strcmpi(config.mode, 'edge') && groundPtCloud.Count > 100000
            ptCloudRoadOnly = groundPtCloud;
        else
            try
                searchRadius = single(0.50);
                [labels, numClusters] = pcsegdist(groundPtCloud, searchRadius);

                if numClusters > 1
                    clusterCounts = histcounts(labels, 1:(numClusters+1));
                    [~, largestClusterIdx] = max(clusterCounts);
                    ptCloudRoadOnly = select(groundPtCloud, labels == largestClusterIdx);
                else
                    ptCloudRoadOnly = groundPtCloud;
                end
            catch
                ptCloudRoadOnly = groundPtCloud;
            end
        end
    else
        ptCloudRoadOnly = groundPtCloud;
    end

    % --- 5. VISUALIZATION (HEADLESS SAFE) ---
    if isfield(config, 'visible') && config.visible && ~strcmpi(config.mode, 'edge')
        fig = figure('Name', 'Stage 4A: SMRF Ground Segmentation', 'Color', 'w');
        ax1 = subplot(1, 2, 1, 'Parent', fig);
        pcshow(ptCloudInput, 'Parent', ax1);
        title(ax1, sprintf('Input ROI Cloud (%d pts)', ptCloudInput.Count), 'Color', 'k');

        ax2 = subplot(1, 2, 2, 'Parent', fig);
        pcshow(ptCloudRoadOnly, 'Parent', ax2);
        title(ax2, sprintf('Extracted Ground Surface (%d pts)', ptCloudRoadOnly.Count), 'Color', 'k');
        linkaxes([ax1, ax2]);
        drawnow;
    end
end