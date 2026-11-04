function [ptCloudCleanRoad] = step5_roadCutter(ptCloudIn, config)
    % ROBUSTROADCUTTER - Universal, scenario-resistant road boundary trimmer.
    % Handles curved bends, undulating Z-heights, attached grass/dirt, and varying widths.
    %
    % Inputs:
    %   ptCloudIn - Input road cloud (PCA-aligned: X = longitudinal, Y = lateral)
    %   config    - Centralized pipeline configuration struct
    %
    % Outputs:
    %   ptCloudCleanRoad - Pure, isolated road surface point cloud (single precision)

    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    if ptCloudIn.Count == 0, ptCloudCleanRoad = ptCloudIn; return; end

    fprintf('\n------------------------------------------------------------\n');
    fprintf('>>> UNIVERSAL ROAD CUTTER: Processing Complex Geometry...\n');
    fprintf('------------------------------------------------------------\n');
    tic;

    xyz = single(ptCloudIn.Location);
    x = xyz(:, 1); y = xyz(:, 2); z = xyz(:, 3);
    numPoints = ptCloudIn.Count;

    % =========================================================================
    % LAYER 1: LOCAL 2D DETRENDING (Handles Macro Undulations & Vertical Bumps)
    % =========================================================================
    % Flattens hills/dips by calculating Z relative to local 0.5m x 0.5m ground min
    detrendCellSize = single(0.50);
    xBinDetrend = floor((x - min(x)) / detrendCellSize) + 1;
    yBinDetrend = floor((y - min(y)) / detrendCellSize) + 1;
    [~, ~, cellIC] = unique([xBinDetrend, yBinDetrend], 'rows');

    zMinPerCell = accumarray(cellIC, z, [], @min);
    deltaZ = z - zMinPerCell(cellIC);

    % Keep points within 6 cm of local cell ground baseline
    maxAllowableBump = single(0.06); 
    heightMask = (deltaZ <= maxAllowableBump);
    fprintf('  -> [Layer 1] Height Detrending: Retained %d / %d pts (bump <= %.2fm).\n', ...
        sum(heightMask), numPoints, maxAllowableBump);

    ptCloudLayer1 = select(ptCloudIn, heightMask);
    if ptCloudLayer1.Count == 0, ptCloudLayer1 = ptCloudIn; end

    % =========================================================================
    % LAYER 2: MATERIAL GAPPING (ExG Vegetation & Intensity Drop)
    % =========================================================================
    % Drops non-asphalt materials to create physical spatial gaps along Y-edges
    xyz1 = single(ptCloudLayer1.Location);
    materialMask = true(ptCloudLayer1.Count, 1);

    % Color / Vegetation Filter (Excess Green Index)
    if isprop(ptCloudLayer1, 'Color') && ~isempty(ptCloudLayer1.Color)
        rgb = single(ptCloudLayer1.Color);
        R = rgb(:,1); G = rgb(:,2); B = rgb(:,3);
        exg = (2 * G - R - B) ./ (R + G + B + 1e-5);
        
        vegMask = (exg > single(0.12));
        materialMask = materialMask & ~vegMask;
        fprintf('     * [ExG Color] Dropped %d vegetation points.\n', sum(vegMask));
    end

    % Intensity Filter (Drop extreme dark/bright non-asphalt returns)
    if isprop(ptCloudLayer1, 'Intensity') && ~isempty(ptCloudLayer1.Intensity)
        intensity = single(ptCloudLayer1.Intensity);
        qLow  = prctile(intensity, 2);
        qHigh = prctile(intensity, 98);
        intensityMask = (intensity >= qLow) & (intensity <= qHigh);
        materialMask = materialMask & intensityMask;
    end

    ptCloudLayer2 = select(ptCloudLayer1, materialMask);
    if ptCloudLayer2.Count == 0, ptCloudLayer2 = ptCloudLayer1; end

    % =========================================================================
    % LAYER 3: DYNAMIC CURVED TRAJECTORY & WIDTH ENVELOPE TRACKING
    % =========================================================================
    % Slices along X, tracks peak Y-density to follow S-curves and turns
    xyz2 = single(ptCloudLayer2.Location);
    x2 = xyz2(:, 1); y2 = xyz2(:, 2);

    sliceWidthX = single(0.50); % 50 cm longitudinal slices
    yBinWidth   = single(0.15); % 15 cm lateral bins

    xMin = min(x2); xMax = max(x2);
    numSlices = ceil((xMax - xMin) / sliceWidthX);

    yLeftBound    = zeros(numSlices, 1, 'single');
    yRightBound   = zeros(numSlices, 1, 'single');
    sliceCentersX = zeros(numSlices, 1, 'single');

    for i = 1:numSlices
        xStart = xMin + (i-1) * sliceWidthX;
        xEnd   = xStart + sliceWidthX;
        sliceCentersX(i) = (xStart + xEnd) / single(2.0);

        sliceMask = (x2 >= xStart) & (x2 < xEnd);
        ySlice = y2(sliceMask);

        if length(ySlice) < 25
            yLeftBound(i)  = NaN;
            yRightBound(i) = NaN;
            continue;
        end

        % 1D Density Histogram along Y-axis
        ySliceMin = min(ySlice); ySliceMax = max(ySlice);
        binEdges = ySliceMin : yBinWidth : ySliceMax;

        if length(binEdges) < 3
            yLeftBound(i)  = ySliceMin;
            yRightBound(i) = ySliceMax;
            continue;
        end

        counts = histcounts(ySlice, binEdges);
        binCenters = binEdges(1:end-1) + (yBinWidth / single(2.0));

        % Peak asphalt density center
        [~, maxPeakIdx] = max(counts);
        densityThresh = single(0.15) * counts(maxPeakIdx); % Drop sparse side-lobes
        denseBins = find(counts >= densityThresh);

        peakClusterIdx = find(denseBins == maxPeakIdx);
        if isempty(peakClusterIdx)
            yLeftBound(i)  = ySliceMin;
            yRightBound(i) = ySliceMax;
            continue;
        end

        % Trace contiguous asphalt body left and right from peak
        leftIdx = peakClusterIdx;
        while leftIdx > 1 && (denseBins(leftIdx) - denseBins(leftIdx-1) == 1)
            leftIdx = leftIdx - 1;
        end

        rightIdx = peakClusterIdx;
        while rightIdx < length(denseBins) && (denseBins(rightIdx+1) - denseBins(rightIdx) == 1)
            rightIdx = rightIdx + 1;
        end

        yLeftBound(i)  = binCenters(denseBins(leftIdx));
        yRightBound(i) = binCenters(denseBins(rightIdx));
    end

    % Smooth trajectory envelope using moving median (3m window along road)
    validSlices = ~isnan(yLeftBound);
    if sum(validSlices) >= 5
        yLeftInterp  = interp1(sliceCentersX(validSlices), yLeftBound(validSlices),  sliceCentersX, 'linear', 'extrap');
        yRightInterp = interp1(sliceCentersX(validSlices), yRightBound(validSlices), sliceCentersX, 'linear', 'extrap');

        smoothWindow = max(3, round(3.0 / sliceWidthX));
        yLeftSmooth  = movmedian(yLeftInterp,  smoothWindow);
        yRightSmooth = movmedian(yRightInterp, smoothWindow);

        % Trim points outside trajectory envelope
        sliceIndices = floor((x2 - xMin) / sliceWidthX) + 1;
        sliceIndices = min(max(sliceIndices, 1), numSlices);

        boundLeft  = yLeftSmooth(sliceIndices);
        boundRight = yRightSmooth(sliceIndices);

        corridorMask = (y2 >= boundLeft) & (y2 <= boundRight);
        ptCloudLayer3 = select(ptCloudLayer2, corridorMask);
        fprintf('  -> [Layer 3] Curve Envelope Tracking: Retained %d / %d pts.\n', ...
            ptCloudLayer3.Count, ptCloudLayer2.Count);
    else
        ptCloudLayer3 = ptCloudLayer2;
    end

    % =========================================================================
    % LAYER 4: EUCLIDEAN CLUSTER SEVERING (Drops Detached Roadside Chunks)
    % =========================================================================
    % Severing bridges allows pcsegdist to cleanly drop outer noise clusters
    try
        searchRadius = single(0.25); % 25 cm search connectivity
        [labelsCluster, numClusters] = pcsegdist(ptCloudLayer3, searchRadius);

        if numClusters > 1
            clusterSizes = histcounts(labelsCluster, 1:(numClusters+1));
            [~, largestClusterIdx] = max(clusterSizes);
            
            cleanIndices = (labelsCluster == largestClusterIdx);
            ptCloudCleanRoad = select(ptCloudLayer3, cleanIndices);
            fprintf('  -> [Layer 4] Cluster Severing: Isolated main road body (%d pts, %d clusters dropped).\n', ...
                ptCloudCleanRoad.Count, numClusters - 1);
        else
            ptCloudCleanRoad = ptCloudLayer3;
        end
    catch ME
        warning('PipelineWarning:ClusterFailed', 'Cluster severing failed (%s). Returning current cloud.', ME.message);
        ptCloudCleanRoad = ptCloudLayer3;
    end

    elapsedTime = toc;
    fprintf('  -> Universal Road Cutting Complete in %.2f seconds (Final: %d pts).\n', ...
        elapsedTime, ptCloudCleanRoad.Count);

    % =========================================================================
    % HEADLESS-SAFE VISUALIZATION
    % =========================================================================
    hasDisplay = ~isempty(java.awt.GraphicsEnvironment.isHeadless) && ~java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment().isHeadless();
    if isfield(config, 'visible') && config.visible && ~strcmpi(config.mode, 'edge') && hasDisplay
        figure('Name', 'Stage 5: Universal Road Cutter Results', 'Color', 'w');
        ax1 = subplot(1, 2, 1);
        pcshow(ptCloudIn, 'Parent', ax1);
        title(ax1, sprintf('Input Road Cloud (%d pts)', ptCloudIn.Count), 'Color', 'k');
        
        ax2 = subplot(1, 2, 2);
        pcshow(ptCloudCleanRoad, 'Parent', ax2);
        title(ax2, sprintf('Final Clean Road Ribbon (%d pts)', ptCloudCleanRoad.Count), 'Color', 'k');
        linkaxes([ax1, ax2]);
        drawnow;
    end
end