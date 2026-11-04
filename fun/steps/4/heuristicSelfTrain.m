function [ptCloudGround, classModel, bestThreshold, groundIndices] = heuristicSelfTrain(ptCloudRaw, ...
    methodRef, binSize, macroSize, profileMode, minSearchHeight, K1, K2, maxNumSplits, greenThreshold, enablePlot)
    % HEURISTICSELFTRAIN - Valley Detection + Environment-Aware Profile Adaptation
    % Optimized for high-performance execution on edge devices (Jetson Nano) and laptops.
    
    if nargin < 1, error('Provide at least the raw point cloud (ptCloudRaw).'); end
    if nargin < 2 || isempty(methodRef),      methodRef = 'min'; end
    if nargin < 3 || isempty(binSize),        binSize = 0.1; end
    if nargin < 4 || isempty(macroSize),      macroSize = 25.0; end
    if nargin < 5 || isempty(profileMode),    profileMode = 'road'; end
    if nargin < 6 || isempty(minSearchHeight),minSearchHeight = []; end 
    if nargin < 7 || isempty(K1),             K1 = 15; end                
    if nargin < 8 || isempty(K2),             K2 = 60; end                
    if nargin < 9 || isempty(maxNumSplits),   maxNumSplits = 35; end      
    if nargin < 10 || isempty(greenThreshold),greenThreshold = []; end  
    if nargin < 11 || isempty(enablePlot),    enablePlot = false; end     

    % --- PROFILE-BASED ENVIRONMENT ADAPTATION ---
    % Automatically adjust default thresholds based on scene category if not explicitly provided
    switch lower(profileMode)
        case 'forest'
            if isempty(minSearchHeight), minSearchHeight = 0.18; end % Higher search to clear undergrowth
            if isempty(greenThreshold),  greenThreshold  = 0.10; end % Lower green sensitivity for dense foliage
            fprintf('  -> [PROFILE] Environment: FOREST (Adjusted minSearchHeight=%.2f, greenThresh=%.2f)\n', minSearchHeight, greenThreshold);
        case {'town', 'urban'}
            if isempty(minSearchHeight), minSearchHeight = 0.08; end % Lower search for curbs and sidewalks
            if isempty(greenThreshold),  greenThreshold  = 0.25; end % Higher green threshold (less vegetation priority)
            fprintf('  -> [PROFILE] Environment: TOWN/URBAN (Adjusted minSearchHeight=%.2f, greenThresh=%.2f)\n', minSearchHeight, greenThreshold);
        otherwise % 'road' or default
            if isempty(minSearchHeight), minSearchHeight = 0.12; end
            if isempty(greenThreshold),  greenThreshold  = 0.15; end
            fprintf('  -> [PROFILE] Environment: ROAD/STANDARD (Using standard parameters:minSearchHeight=%.2f, greenThresh=%.2f)\n', minSearchHeight, greenThreshold);
    end

    xyz = single(ptCloudRaw.Location);
    x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);
    numPoints = length(z);
    
    % --- STEP 0: EXTRACT RGB COLOR & METADATA (SINGLE PRECISION) ---
    hasColor = false;
    exg = zeros(numPoints, 1, 'single');
    try
        if ~isempty(ptCloudRaw.Color)
            rgb = single(ptCloudRaw.Color);
            R = rgb(:,1); G = rgb(:,2); B = rgb(:,3);
            exg = (2 * G - R - B) ./ (R + G + B + 1e-5); 
            hasColor = true;
        end
    catch
    end
    
    hasReturns = false;
    returnNumber = ones(numPoints, 1, 'single');
    try
        if ~isempty(ptCloudRaw.ReturnNumber)
            returnNumber = single(ptCloudRaw.ReturnNumber);
            hasReturns = true;
        end
    catch
    end
    
    hasIntensity = false;
    intensity = zeros(numPoints, 1, 'single');
    try
        if ~isempty(ptCloudRaw.Intensity)
            intensity = single(ptCloudRaw.Intensity);
            hasIntensity = true;
        end
    catch
    end

    % -----------------------------------------------------------------
    % PHASE 1: ADAPTIVE PSEUDO-LABELING (Valley Detection + ExG Threshold)
    % -----------------------------------------------------------------
    histBinSize    = binSize * 0.1;        
    maxHeightLimit = 0.80;                 
    
    xMin = min(x); yMin = min(y);
    validMask = ~isnan(z);
    validIndices = find(validMask);
    numValid = length(validIndices);
    
    xMacroIdx = floor((x - xMin) / macroSize) + 1;
    yMacroIdx = floor((y - yMin) / macroSize) + 1;
    
    [macroUnique, ~, macroIC] = unique([xMacroIdx(validMask), yMacroIdx(validMask)], 'rows');
    numTiles = size(macroUnique, 1);
    
    heightAboveGround  = zeros(numValid, 1, 'single');
    featRoughnessValid = zeros(numValid, 1, 'single');
    tempLabels         = ones(numValid, 1, 'single'); % 0 = Ground, 1 = Obstacle
    tileThresholds     = zeros(numTiles, 1, 'single');
    
    validZ   = z(validMask);
    validX   = x(validMask);
    validY   = y(validMask);
    validExG = exg(validMask);
    
    for t = 1:numTiles
        tilePointMask = (macroIC == t);
        if sum(tilePointMask) < 200, continue; end
        
        tileZ_sub = validZ(tilePointMask);
        tileX_sub = validX(tilePointMask);
        tileY_sub = validY(tilePointMask);
        
        tileXMin = min(tileX_sub); tileYMin = min(tileY_sub);
        tileXBin = floor((tileX_sub - tileXMin) / binSize) + 1;
        tileYBin = floor((tileY_sub - tileYMin) / binSize) + 1;
        [~, ~, tileIC_sub] = unique([tileXBin, tileYBin], 'rows');
        
        switch lower(methodRef)
            case 'prctile5'
                zRefPerBin = accumarray(tileIC_sub, tileZ_sub, [], @(vals) prctile(vals, 5));
            case 'min'
                zRefPerBin = accumarray(tileIC_sub, tileZ_sub, [], @min);
            otherwise
                zRefPerBin = accumarray(tileIC_sub, tileZ_sub, [], @min);
        end
        
        tileMinZ_sub = zRefPerBin(tileIC_sub);
        tileHeight = tileZ_sub - tileMinZ_sub;
        heightAboveGround(tilePointMask) = tileHeight;
        
        zMeanPerBin = accumarray(tileIC_sub, tileZ_sub, [], @mean);
        tileRoughness = abs(tileZ_sub - zMeanPerBin(tileIC_sub));
        featRoughnessValid(tilePointMask) = tileRoughness;
        
        zMaxPerBin = accumarray(tileIC_sub, tileZ_sub, [], @max);
        zMinPerBin = accumarray(tileIC_sub, tileZ_sub, [], @min);
        zRangePerBin = zMaxPerBin - zMinPerBin;
        tileBinRange = zRangePerBin(tileIC_sub);
        
        % Local Valley Detection via Histogram Smoothing
        [counts, binEdges] = histcounts(tileHeight, 0 : histBinSize : maxHeightLimit);
        binCenters = binEdges(1:end-1) + (histBinSize / 2);
        
        if sum(counts) > 50
            [~, groundPeakIdx] = max(counts); 
            minIdxLimit = find(binCenters >= minSearchHeight, 1, 'first');
            if isempty(minIdxLimit), minIdxLimit = groundPeakIdx + 1; end
            
            effectiveStartIdx = max(groundPeakIdx, minIdxLimit);
            searchZone = counts(effectiveStartIdx : end);
            
            if length(searchZone) > 2
                smoothedZone = movmean(searchZone, 3);
                [~, minRelativeIdx] = min(smoothedZone);
                tileThresh = binCenters(effectiveStartIdx + minRelativeIdx - 1);
            else
                tileThresh = 0.25;
            end
        else
            tileThresh = 0.25;
        end
        
        tileThresh = max(0.1, min(tileThresh, 0.3));
        tileThresholds(t) = tileThresh;
        
        % --- GEOMETRIC & CHROMATIC PSEUDO-LABELING ---
        localRoughnessLimit = binSize * 0.25;  
        localBinRangeLimit  = binSize * 0.35;  
        
        isAboveThreshold    = (tileHeight >= tileThresh); 
        isVegetationClutter = (tileBinRange > localBinRangeLimit) & (tileHeight > 0.02); 
        isRoughSurface      = (tileRoughness > localRoughnessLimit) & (tileHeight > 0.05);
        isGreenVegetation   = hasColor & (validExG(tilePointMask) > greenThreshold);
        
        isObstacle = isAboveThreshold | isVegetationClutter | isRoughSurface | isGreenVegetation;
        
        % Smooth asphalt protection (prevents holes on flat road surfaces)
        strictSmoothness = (tileRoughness < (binSize * 0.15)) & ...
                           (tileBinRange < (binSize * 0.15)) & ...
                           (tileHeight < (tileThresh * 0.4));
                       
        isObstacle(strictSmoothness) = false; 
        
        subIndices = find(tilePointMask);
        subLabels  = zeros(length(subIndices), 1, 'single'); 
        subLabels(isObstacle) = 1; % 1 = Obstacle
        tempLabels(subIndices) = subLabels;
    end
    
    bestThreshold = median(tileThresholds(tileThresholds > 0));
    if isnan(bestThreshold), bestThreshold = 0.25; end
    
    % -----------------------------------------------------------------
    % PHASE 2: FEATURE ENGINEERING & MULTISCALE K-NN (VECTORIZED)
    % -----------------------------------------------------------------
    validXYZSubset = xyz(validMask, :);
    kdtree = KDTreeSearcher(validXYZSubset);
    
    [idxK1, ~] = knnsearch(kdtree, validXYZSubset, 'K', min(double(K1), numValid));
    [idxK2, ~] = knnsearch(kdtree, validXYZSubset, 'K', min(double(K2), numValid));
    
    neighborZ_K1 = validZ(idxK1);
    featGradientK1 = max(abs(validZ - neighborZ_K1), [], 2);
    
    neighborZ_K2 = validZ(idxK2);
    featGradientK2 = max(abs(validZ - neighborZ_K2), [], 2);
    
    baseFeatures = [heightAboveGround, featRoughnessValid, featGradientK1, featGradientK2];
    if hasIntensity, baseFeatures = [baseFeatures, intensity(validMask)]; end
    if hasColor, baseFeatures = [baseFeatures, validExG]; end
    if hasReturns, baseFeatures = [baseFeatures, returnNumber(validMask)]; end
    
    X_features = baseFeatures;
    y_targets = tempLabels;
    
    % -----------------------------------------------------------------
    % PHASE 3: SUPERVISED DECISION TREE TRAINING
    % -----------------------------------------------------------------
    classModel = fitctree(X_features, y_targets, ...
        'MaxNumSplits', double(maxNumSplits), ...
        'Prior', 'uniform');
    
    % -----------------------------------------------------------------
    % PHASE 4: INFERENCE & PREDICTION
    % -----------------------------------------------------------------
    predictedLabels = predict(classModel, X_features);
    
    groundMask = false(numPoints, 1);
    validGroundSubset = (predictedLabels == 0); % 0 = Ground
    groundIndices = validIndices(validGroundSubset);
    groundMask(groundIndices) = true;
    
    if isa(ptCloudRaw, 'pointCloud')
        ptCloudGround = select(ptCloudRaw, groundMask);
    else
        ptCloudGround = pointCloud(xyz(groundMask, :));
    end
    
    % -----------------------------------------------------------------
    % PHASE 5: CONDITIONAL VISUALIZATION (HEADLESS SAFE)
    % -----------------------------------------------------------------
    hasDisplay = ~isempty(java.awt.GraphicsEnvironment.isHeadless) && ~java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment().isHeadless();
    if enablePlot && hasDisplay
        figure('Name', 'Extracted Road Core - Pro Pipeline', 'Color', 'w');
        xyz_ground = double(ptCloudGround.Location);
        if ~isempty(xyz_ground)
            pcshow(xyz_ground, xyz_ground(:,3), 'MarkerSize', 15);
            title(sprintf('Extracted Road Core (%s) | Pts: %d', profileMode, ptCloudGround.Count), 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            colormap jet; colorbar; 
            grid on; axis equal; view(3);
            drawnow;
        end
    end
end