function [ptCloudGround, classModel, bestThreshold, groundIndices] = heuristicSelfTrain(ptCloudRaw,...
    methodRef, binSize, macroSize, profileMode, minSearchHeight, K1, K2, maxNumSplits, greenThreshold, enablePlot)
% HEURISTICSELFTRAIN - Valley Detection + Color Filter Explicit
% Eliminates slow for-loops to ensure high-performance execution on edge devices (Jetson Nano).

    if nargin < 1, error('Provide at least the raw point cloud (ptCloudRaw).'); end
    if nargin < 2 || isempty(methodRef),      methodRef = 'min'; end
    if nargin < 3 || isempty(binSize),        binSize = 0.1; end
    if nargin < 4 || isempty(macroSize),      macroSize = 25.0; end
    if nargin < 5 || isempty(profileMode),    profileMode = 'road'; end
    if nargin < 6 || isempty(minSearchHeight),minSearchHeight = 0.12; end 
    if nargin < 7 || isempty(K1),             K1 = 15; end                
    if nargin < 8 || isempty(K2),             K2 = 60; end                
    if nargin < 9 || isempty(maxNumSplits),   maxNumSplits = 35; end      
    if nargin < 10 || isempty(greenThreshold),greenThreshold = 0.15; end  
    if nargin < 11 || isempty(enablePlot),    enablePlot = false; end     

    xyz = double(ptCloudRaw.Location);
    x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);
    numPoints = length(z);
    
    % --- STEP 0: EXTRACT RGB COLOR & METADATA ---
    hasColor = false;
    exg = zeros(numPoints, 1);
    try
        if ~isempty(ptCloudRaw.Color)
            rgb = double(ptCloudRaw.Color);
            R = rgb(:,1); G = rgb(:,2); B = rgb(:,3);
            % Excess Green (ExG) index to isolate vegetation
            exg = (2 * G - R - B) ./ (R + G + B + 1e-5); 
            hasColor = true;
        end
    catch, end
    
    hasReturns = false;
    returnNumber = ones(numPoints, 1);
    try
        if ~isempty(ptCloudRaw.ReturnNumber)
            returnNumber = double(ptCloudRaw.ReturnNumber);
            hasReturns = true;
        end
    catch, end
    
    hasIntensity = false;
    intensity = zeros(numPoints, 1);
    try
        if ~isempty(ptCloudRaw.Intensity)
            intensity = double(ptCloudRaw.Intensity);
            hasIntensity = true;
        end
    catch, end

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
    
    heightAboveGround  = zeros(numValid, 1);
    featRoughnessValid = zeros(numValid, 1);
    tempLabels         = ones(numValid, 1); 
    tileThresholds     = zeros(numTiles, 1);
    
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
        
        % Local Valley Detection
        [counts, binEdges] = histcounts(tileHeight, 0 : histBinSize : maxHeightLimit);
        binCenters = binEdges(1:end-1) + (histBinSize / 2);
        
        if sum(counts) > 50
            [~, groundPeakIdx] = max(counts); 
            minIdxLimit = find(binCenters >= minSearchHeight, 1, 'first');
            if isempty(minIdxLimit), minIdxLimit = groundPeakIdx + 1; end
            
            effectiveStartIdx = max(groundPeakIdx, minIdxLimit);
            searchZone = counts(effectiveStartIdx : end);
            
            if length(searchZone) > 2
                [~, minRelativeIdx] = min(searchZone);
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
        subLabels  = zeros(length(subIndices), 1); 
        subLabels(isObstacle) = 1; 
        tempLabels(subIndices) = subLabels;
    end
    
    bestThreshold = median(tileThresholds(tileThresholds > 0));
    if isnan(bestThreshold), bestThreshold = 0.25; end
    
    % -----------------------------------------------------------------
    % PHASE 2: FEATURE ENGINEERING & MULTISCALE K-NN (FULLY VECTORIZED)
    % -----------------------------------------------------------------
    validXYZ = xyz(validMask, :);
    kdtree = KDTreeSearcher(validXYZ);
    
    [idxK1, ~] = knnsearch(kdtree, validXYZ, 'K', min(double(K1), numValid));
    [idxK2, ~] = knnsearch(kdtree, validXYZ, 'K', min(double(K2), numValid));
    
    % Fully vectorized gradient calculation (eliminates slow for-loops)
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
    validGroundSubset = (predictedLabels == 0);
    groundIndices = validIndices(validGroundSubset);
    groundMask(groundIndices) = true;
    
    if isa(ptCloudRaw, 'pointCloud')
        ptCloudGround = select(ptCloudRaw, groundMask);
    else
        ptCloudGround = pointCloud(xyz(groundMask, :));
    end
    
    % -----------------------------------------------------------------
    % PHASE 5: CONDITIONAL VISUALIZATION
    % -----------------------------------------------------------------
    if enablePlot
        figure('Name', 'Extracted Road Core - Pro Pipeline', 'Color', 'w');
        xyz_ground = double(ptCloudGround.Location);
        pcshow(xyz_ground, xyz_ground(:,3), 'MarkerSize', 15);
        title(sprintf('Extracted Road Core (Pro) | Points: %d', ptCloudGround.Count), 'FontSize', 12, 'FontWeight', 'bold');
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        colormap jet; colorbar; 
        grid on; axis equal; view(3);
    end
end






% Matlab
% 
% % =================================================================
% % FUZZY LOGIC ADAPTIVE PSEUDO-LABELING
% % =================================================================
% % Define dynamic fuzzy scaling based on local bin size and thresholds
% maxAllowableRoughness = binSize * 0.60;  % Tolerant to off-road bumps
% maxAllowableBinRange  = binSize * 0.80;  % Tolerant to minor vegetation clutter
% 
% % 1. FUZZY MEMBERSHIP: HEIGHT (0 = High Object, 1 = Ground Level)
% % Uses a smooth linear ramp dropping to 0 as height approaches tileThresh
% mu_height = max(0, min(1, (tileThresh - tileHeight) / (tileThresh * 0.4 + 1e-5)));
% 
% % 2. FUZZY MEMBERSHIP: ROUGHNESS (0 = Very Rough, 1 = Perfectly Smooth)
% mu_roughness = max(0, min(1, (maxAllowableRoughness - tileRoughness) / (maxAllowableRoughness + 1e-5)));
% 
% % 3. FUZZY MEMBERSHIP: BIN RANGE (0 = High Vertical Scatter, 1 = Flat Bin)
% mu_binRange = max(0, min(1, (maxAllowableBinRange - tileBinRange) / (maxAllowableBinRange + 1e-5)));
% 
% % 4. FUZZY MEMBERSHIP: CHROMATIC / VEGETATION (0 = Green Plant, 1 = Earth/Road)
% if hasColor
%     currentExG = validExG(tilePointMask);
%     % Smooth transition around greenThreshold
%     mu_color = max(0, min(1, (greenThreshold - currentExG) / 0.10 + 0.5));
% else
%     mu_color = ones(size(tileHeight)); % Neutral if no RGB data available
% end
% 
% % =================================================================
% % FUZZY INFERENCE ENGINE (Mamdani Min-Operator Rule Aggregation)
% % =================================0================================
% % Rule: A point is GROUND if it is (Low Height) AND (Smooth Surface) 
% %       AND (Low Bin Range) AND (Non-Green Color).
% groundConfidence = min([mu_height, mu_roughness, mu_binRange, mu_color], [], 2);
% 
% % Soft thresholding: Confidence < 0.45 means it's an obstacle (1), else ground (0)
% isObstacle = (groundConfidence < 0.45);
% 
% % Smooth asphalt / hard-packed dirt floor safeguard
% strictSmoothness = (tileRoughness < (binSize * 0.20)) & (tileHeight < (tileThresh * 0.5));
% isObstacle(strictSmoothness) = false; % Force override as ground
% 
% subIndices = find(tilePointMask);
% subLabels  = zeros(length(subIndices), 1); 
% subLabels(isObstacle) = 1; 
% tempLabels(subIndices) = subLabels;