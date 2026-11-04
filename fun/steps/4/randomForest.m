% FIXED SIGNATURE: Replaced bestParameters with rfModel
function [ptCloudRoad, model] = randomForest(ptCloudRoadOnly, ptCloudClean, config, rfModel)

fprintf('\n--------------------------------------------\n');
fprintf('MACHINE LEARNING: Random Forest (Parallel)\n');
fprintf('--------------------------------------------\n');

if nargin < 4, rfModel = []; end
if nargin < 3 || isempty(config), config = struct(); end
if ~isfield(config, 'defaultConfigs'), config.defaultConfigs = fullfile('files', 'configs'); end
if ~isfield(config, 'searchRadius'), config.searchRadius = 0.5; end
if ~isfield(config, 'minTrainPts'), config.minTrainPts = 1000; end
if ~isfield(config, 'optimizeFilter'), config.optimizeFilter = true; end
if ~isfield(config, 'kNeighbors'), config.kNeighbors = 20; end 

tic;

% --- STEP 1: UNIFIED CACHE MANAGEMENT ---
cacheFile = fullfile(config.defaultConfigs, 'heuristic_rf_cache.mat');

% FIXED: Safe check for existing model object
if ~isempty(rfModel) && isa(rfModel, 'classreg.learning.classif.ClassificationEnsemble')
    fprintf('  -> Using pre-optimized Random Forest model provided in arguments...\n');
    model = rfModel;
else
    if exist(cacheFile, 'file') && ~config.optimizeFilter
        fprintf('  -> Loading pre-trained Random Forest model from unified cache (%s)...\n', cacheFile);
        savedData = load(cacheFile, 'trained_rf');
        model = savedData.trained_rf;
    else
        fprintf('  -> Initiating automated parallel training using SMRF Master Teacher...\n');
        % Training local function Call
        model = trainRoadClassifier(ptCloudRoadOnly, ptCloudClean, config);

        configDir = fileparts(cacheFile);
        if ~isfolder(configDir), mkdir(configDir); end

        trained_rf = model;
        if exist(cacheFile, 'file')
            save(cacheFile, 'trained_rf', '-append');
        else
            save(cacheFile, 'trained_rf');
        end
    end
end

% --- STEP 2: FULL-SCALE FEATURE ENGINEERING ---
fprintf('  -> Extracting radius-based geometric features (Radius = %.2f m, k = %d)...\n', ...
    config.searchRadius, config.kNeighbors);
[X_features] = extractRobustFeatures(ptCloudClean, config.kNeighbors, config.searchRadius, config);

% --- STEP 3: CLASSIFICATION INFERENCE ---
fprintf('  -> Executing Random Forest ensemble prediction on full dataset...\n');
predictedLabels = predict(model, X_features);
isRoad = (predictedLabels == true);
ptCloudRoad = select(ptCloudClean, isRoad);

elapsedTime = toc;
fprintf('  -> Parallel ML filtering completed in %.2f seconds. Road points extracted: %d\n', ...
    elapsedTime, ptCloudRoad.Count);
end

%% =========================================================================
% MODEL TRAINING (Parallelized Ensemble)
% =========================================================================
function [model] = trainRoadClassifier(ptCloudRoadOnly, ptCloudClean, config, bestParameters)
    fprintf('--- STARTING TRAINING (Safe Mode) ---\n');
    tTotal = tic;
    
    % --- 0. SANITIZE POINT CLOUD ---
    locs = ptCloudClean.Location;
    validRows = ~any(isnan(locs) | isinf(locs), 2);
    if ~all(validRows)
        fprintf('  -> Warning: Removing %d invalid points (NaN/Inf)...\n', sum(~validRows));
        ptCloudClean = select(ptCloudClean, validRows);
    end
   
    % 1. Feature Extraction on the FULL Dense Cloud
    tStage = tic;
    X_features_full = extractRobustFeatures(ptCloudClean, config.kNeighbors, config.searchRadius, config);
    fprintf('  [TIME] Feature Extraction (Full Density): %.2f seconds\n', toc(tStage));
    numPoints = ptCloudClean.Count;
    locations = double(ptCloudClean.Location);
    
    % 2. Label Mapping
    tStage = tic;
    Y_labels_full = false(numPoints, 1);
    if ptCloudRoadOnly.Count > 0
        roadLocations = double(ptCloudRoadOnly.Location);
        [nearestIdx, dist] = knnsearch(locations, roadLocations);
        validMatch = dist < 1e-2; 
        Y_labels_full(nearestIdx(validMatch)) = true;
    end
    fprintf('  [TIME] KNN Label Mapping: %.2f seconds\n', toc(tStage));
    
    % 3. Row Subsampling for Fast Optimization
    maxTrainingPoints = config.minTrainPts; 
    if numPoints > maxTrainingPoints
        fprintf('  -> Subsampling %d rows from feature matrix for rapid optimization...\n', maxTrainingPoints);
        rng(42); 
        trainIndices = randperm(numPoints, maxTrainingPoints);
        X_features = X_features_full(trainIndices, :);
        Y_labels = Y_labels_full(trainIndices);
    else
        X_features = X_features_full;
        Y_labels = Y_labels_full;
    end
    
    % Safe Parallel Pool Check (Disabled on Edge hardware)
    useParallelPool = ~isempty(ver('parallel')) && ~isempty(gcp('nocreate')) && ~strcmpi(config.mode, 'edge');
    
    % 4. Training Logic
    tStage = tic;
    if config.optimizeFilter
        fprintf('  -> Running Bounded Bayesian Optimization (Bagging Only)...\n');
        
        hyperparams = hyperparameters('fitcensemble', X_features, Y_labels, 'Tree');
        methodRow = strcmp({hyperparams.Name}, 'Method');
        hyperparams(methodRow).Optimize = false;
        learnRateRow = strcmp({hyperparams.Name}, 'LearnRate');
        hyperparams(learnRateRow).Optimize = false;
        
        for i = 1:height(hyperparams)
            paramName = hyperparams(i).Name;
            switch paramName
                case 'MinLeafSize'
                    hyperparams(i).Range = [1, 30];  
                    hyperparams(i).Optimize = true;
                case 'MaxNumSplits'
                    hyperparams(i).Range = [20, 1000];
                    hyperparams(i).Optimize = true;
                case 'NumLearningCycles'
                    hyperparams(i).Range = [20, 1000];   
                    hyperparams(i).Optimize = true;
                case 'NumVariablesToSample'
                    hyperparams(i).Optimize = true;
            end
        end
        
        model = fitcensemble(X_features, Y_labels, ...
            'Method', 'Bag', ...
            'OptimizeHyperparameters', hyperparams, ... 
            'HyperparameterOptimizationOptions', struct(...
                'AcquisitionFunction', 'expected-improvement-plus', ...
                'MaxObjectiveEvaluations', 30, ...
                'UseParallel', useParallelPool, ...
                'Verbose', 0));
    else
        fprintf('  -> Training with previously optimized parameters from cache...\n');
        numCycles = 100; minLeaf = 5; maxSplits = 100; numVarsToSample = 'all';
        if nargin >= 4 && ~isempty(bestParameters)
            if isfield(bestParameters, 'NumLearningCycles'), numCycles = bestParameters.NumLearningCycles; end
            if isfield(bestParameters, 'MinLeafSize'), minLeaf = bestParameters.MinLeafSize; end
            if isfield(bestParameters, 'MaxNumSplits'), maxSplits = bestParameters.MaxNumSplits; end
            if isfield(bestParameters, 'NumVariablesToSample'), numVarsToSample = bestParameters.NumVariablesToSample; end
        end
        
        model = fitcensemble(X_features, Y_labels, ...
            'Method', 'Bag', ...
            'NumLearningCycles', numCycles, ...
            'MinLeafSize', minLeaf, ...
            'MaxNumSplits', maxSplits, ...
            'NumVariablesToSample', numVarsToSample, ...
            'Options', statset('UseParallel', useParallelPool));
    end
    
    fprintf('  [TIME] Model Training Phase: %.2f seconds\n', toc(tStage));
    fprintf('--- TOTAL TRAINING TIME: %.2f seconds ---\n', toc(tTotal));
    fprintf('  -> Optimal Random Forest model trained successfully!\n');
end

%% =========================================================================
% FEATURE EXTRACTION WITH EXPLICIT LOCAL DENSITY FEATURE
% =========================================================================
function [X_features, featureNames] = extractRobustFeatures(ptCloudIn, kNeighb, densityRadius, config)
    if nargin < 2 || isempty(kNeighb), kNeighb = 30; end
    if nargin < 3 || isempty(densityRadius), densityRadius = 0.5; end 
    if nargin < 4 || isempty(config), config = struct(); end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end
    
    enablePlot = isfield(config, 'visible') && config.visible;
    
    % CAST TO SINGLE IMMEDIATELY TO SAVE RAM & BANDWIDTH
    xyz = single(ptCloudIn.Location);
    numPoints = size(xyz, 1);
    fprintf('  -> Extracting vectorized features (Single Precision) for %d points...\n', numPoints);
    
    kdtree = KDTreeSearcher(xyz);
    z = xyz(:, 3);
    
    % --- 1. BASE GEOMETRIC FEATURES ---
    heightAboveGround = z - min(z);
    
    [idxK1, ~] = knnsearch(kdtree, xyz, 'K', min(kNeighb, numPoints));
    neighborZ = z(idxK1);
    featGradient = max(abs(z - neighborZ), [], 2);
    meanNeighborZ = mean(neighborZ, 2);
    featRoughness = abs(z - meanNeighborZ);
    
    % --- 2. VECTORIZED VERTICALITY ---
    if isprop(ptCloudIn, 'Normal') && ~isempty(ptCloudIn.Normal)
        normals = single(ptCloudIn.Normal); 
    else
        normals = single(pcnormals(ptCloudIn, kNeighb));
    end
    verticality = 1 - abs(normals(:, 3)); 
    
    % --- 3. VECTORIZED LOCAL DENSITY FEATURE ---
    idxRadius = rangesearch(kdtree, xyz, densityRadius);
    localDensity = log(single(cellfun(@numel, idxRadius)) + 1); 
    
    % --- 4. ASSEMBLE FEATURE MATRIX & NAMES ---
    X_features = [ ...
        heightAboveGround, ...
        featRoughness, ...
        featGradient, ...
        verticality, ...
        localDensity ...
        ];
    
    featureNames = { ...
        'HeightAboveGround', ...
        'Roughness', ...
        'Gradient', ...
        'Verticality', ...
        'LocalDensity' ...
        };
    
    if ~isempty(ptCloudIn.Intensity)
        X_features = [X_features, single(ptCloudIn.Intensity)];
        featureNames{end+1} = 'Intensity';
    end
    
    if ~isempty(ptCloudIn.Color)
        rgb = single(ptCloudIn.Color);
        exg = (2 * rgb(:,2) - rgb(:,1) - rgb(:,3)) ./ (sum(rgb, 2) + 1e-5);
        X_features = [X_features, exg];
        featureNames{end+1} = 'ExcessGreen';
    end
    
    X_features = single(X_features);
    fprintf('  -> Feature extraction complete. Total feature dimensions: %d\n', size(X_features, 2));
    
    % --- HEADLESS-SAFE PLOTTING ---
    hasDisplay = ~isempty(java.awt.GraphicsEnvironment.isHeadless) && ~java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment().isHeadless();
    if enablePlot && hasDisplay && ~strcmpi(config.mode, 'edge')
        corrMat = corr(double(X_features), 'Rows', 'pairwise'); 
        figure('Name', 'Feature Correlation Heatmap', 'Color', 'white', 'Position', [100, 100, 800, 600]);
        heatmap(featureNames, featureNames, corrMat, 'Colormap', turbo, 'ColorLimits', [-1, 1]);
        title('Feature Correlation Matrix');
        drawnow;
    end
end