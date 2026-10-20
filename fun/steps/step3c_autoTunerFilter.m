function [ptCloudGround, classModel, bestThreshold, groundIndices] = step3c_autoTunerFilter(...
    ptCloudRaw, methodRef, binSize, macroSize, profileMode, minSearchHeight, K1, K2, maxNumSplits, greenThreshold, enablePlot)
% PIPELINE_ACTIVELEARNINGGROUNDFILTER - Con Valley Detection + Filtro Colore Esplicito

if nargin < 1, error('Fornire almeno la nuvola di punti ptCloudRaw.'); end
if nargin < 2 || isempty(methodRef),      methodRef = 'min'; end
if nargin < 3 || isempty(binSize),        binSize = 0.1; end
if nargin < 4 || isempty(macroSize),      macroSize = 25.0; end
if nargin < 5 || isempty(profileMode),    profileMode = 'road'; end
if nargin < 6 || isempty(minSearchHeight),minSearchHeight = 0.12; end % <-- 6° argomento corretto!
if nargin < 7 || isempty(K1),             K1 = 15; end                % <-- 7°
if nargin < 8 || isempty(K2),             K2 = 60; end                % <-- 8°
if nargin < 9 || isempty(maxNumSplits),   maxNumSplits = 35; end      % <-- 9°
if nargin < 10 || isempty(greenThreshold),greenThreshold = 0.15; end  % <-- 10°
if nargin < 11 || isempty(enablePlot),    enablePlot = false; end     % <-- 11°

    xyz = double(ptCloudRaw.Location);
    x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);
    numPoints = length(z);
    
    % --- 0. ESTRAZIONE COLORE RGB & METADATI ---
    hasColor = false;
    exg = zeros(numPoints, 1);
    try
        if ~isempty(ptCloudRaw.Color)
            rgb = double(ptCloudRaw.Color);
            R = rgb(:,1); G = rgb(:,2); B = rgb(:,3);
            % Indice Excess Green (ExG) per isolare la vegetazione
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
    % FASE 1: PSEUDO-LABELING ADATTIVO (Valley Detection + ExG Threshold)
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
        
        % Valley Detection locale
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
        
        % --- PSEUDO-LABELING GEOMETRICO + CROMATICO ---
        localRoughnessLimit = binSize * 0.25;  
        localBinRangeLimit  = binSize * 0.35;  
        
        isAboveThreshold    = (tileHeight >= tileThresh); 
        isVegetationClutter = (tileBinRange > localBinRangeLimit) & (tileHeight > 0.02); 
        isRoughSurface      = (tileRoughness > localRoughnessLimit) & (tileHeight > 0.05);
        isGreenVegetation   = hasColor & (validExG(tilePointMask) > greenThreshold); % <-- SOGLIA COLORE INTEGRATA!
        
        isObstacle = isAboveThreshold | isVegetationClutter | isRoughSurface | isGreenVegetation;
        
        % Protezione Asfalto Liscio (impedisce buchi sull'asfalto)
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
    % FASE 2: FEATURE ENGINEERING & MULTISCALE K-NN
    % -----------------------------------------------------------------
    validXYZ = xyz(validMask, :);
    kdtree = KDTreeSearcher(validXYZ);
    
    [idxK1, ~] = knnsearch(kdtree, validXYZ, 'K', min(double(K1), numValid));
    [idxK2, ~] = knnsearch(kdtree, validXYZ, 'K', min(double(K2), numValid));
    
    featGradientK1 = zeros(numValid, 1);
    featGradientK2 = zeros(numValid, 1);
    validZ = z(validMask);
    for i = 1:numValid
        featGradientK1(i) = max(abs(validZ(i) - validZ(idxK1(i, :))));
        featGradientK2(i) = max(abs(validZ(i) - validZ(idxK2(i, :))));
    end
    
    baseFeatures = [heightAboveGround, featRoughnessValid, featGradientK1, featGradientK2];
    if hasIntensity, baseFeatures = [baseFeatures, intensity(validMask)]; end
    if hasColor, baseFeatures = [baseFeatures, validExG]; end
    if hasReturns, baseFeatures = [baseFeatures, returnNumber(validMask)]; end
    
    X_features = baseFeatures;
    y_targets = tempLabels;
    
    % -----------------------------------------------------------------
    % FASE 3: DECISION TREE
    % -----------------------------------------------------------------
    classModel = fitctree(X_features, y_targets, ...
        'MaxNumSplits', double(maxNumSplits), ...
        'Prior', 'uniform');
    
    % -----------------------------------------------------------------
    % FASE 4: INFERENZA
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
    % FASE 5: VISUALIZZAZIONE CONDIZIONATA
    % -----------------------------------------------------------------
    if enablePlot
        figure('Name', 'Core Stradale Estratto', 'Color', 'w');
        xyz_ground = double(ptCloudGround.Location);
        scatter3(xyz_ground(:, 1), xyz_ground(:, 2), xyz_ground(:, 3), 12, xyz_ground(:, 3), 'filled');
        title(sprintf('Core Stradale Estratto | Punti: %d', ptCloudGround.Count));
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        colormap jet; colorbar; view(3); grid on; axis equal;
        figure('Name', 'Core Stradale Estratto - Pipeline Pro', 'Color', 'w');
        xyz_ground = double(ptCloudGround.Location);

        % pcshow supporta direttamente la colorazione basata sulla quota Z (o su un vettore colore)
        pcshow(xyz_ground, xyz_ground(:,3), 'MarkerSize', 15);

        title(sprintf('Core Stradale Estratto (Pro) | Punti: %d', ptCloudGround.Count), 'FontSize', 12, 'FontWeight', 'bold');
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        colormap jet; colorbar; 
        grid on; axis equal; view(3);
    end
end