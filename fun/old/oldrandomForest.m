function [ptCloudRoad, model] = oldrandomForest(ptCloudClean, config, modelInput)
% RANDOMFOREST - Fully Vectorized and Accurate Parallel Machine Learning Pipeline
    fprintf('\n============================================\n');
    fprintf('MACHINE LEARNING: Random Forest (Parallel)\n');
    fprintf('============================================\n');
    
    % Spatial bounding boxes safely handled as defaults instead of overwrites
    if ~isfield(config, 'xmax'), config.xmax = 110; end
    if ~isfield(config, 'xmin'), config.xmin = 7.5; end
    if ~isfield(config, 'ymax'), config.ymax = 10.1; end
    if ~isfield(config, 'ymin'), config.ymin = 4.9; end

    tic;
    % --- STEP 0: HANDLE OPTIONAL CONFIG & DEFAULTS ---
    if nargin < 2 || isempty(config)
        config = struct();
    end
    if ~isfield(config, 'defaultConfigs')
        config.defaultConfigs = fullfile('files', 'configs');
    end
    % Replace fixed K-NN with a physical search radius (in meters)
    if ~isfield(config, 'searchRadius')
        config.searchRadius = .5; % 0.5 meters default search radius
    end
    if ~isfield(config, 'minTrainPts')
        config.minTrainPts = 500;
    end
    if ~isfield(config, 'doBayesOpt')
        config.doBayesOpt = true; % Set to false to skip optimization for rapid testing
    end
    
    % --- STEP 1: MODEL MANAGEMENT / TRAINING ---
    modelPath = fullfile(config.defaultConfigs, 'trained_rf.mat');
    hasInputModel = (nargin >= 3 && ~isempty(modelInput));


    %ptCloudClean = pcdownsample(ptCloudClean, 'gridAverage', 0.1);
    indices = 500000:ptCloudClean.Count;
    ptCloudClean = select(ptCloudClean, indices);
    % plotPtCloud3D(ptCloudClean, 'color');

    
    if ~hasInputModel
        if exist(modelPath, 'file')
            fprintf('  -> Loading pre-trained Random Forest model (%s)...\n', modelPath);
            savedData = load(modelPath, 'model');
            model = savedData.model;
        else
            fprintf('  -> No model found. Initiating automated parallel training...\n');
            [model, featureNames] = trainRoadClassifier(ptCloudClean, config);
            
            % View OOB error convergence curve after training
            figure('Name', 'Random Forest OOB Error Convergence', 'Color', 'w');
            oobErrorCurve = oobLoss(model, 'Mode', 'cumulative');
            plot(oobErrorCurve, 'LineWidth', 2);
            xlabel('Number of Trees');
            ylabel('Out-of-Bag Classification Error');
            title('Random Forest Convergence Assessment');
            grid on;
            
            % --- Compute Out-of-Bag Feature Importance ---
            imp = oobPermutedPredictorImportance(model);

            % --- Plot Results ---
            figure('Name', 'Feature Importance Evaluation', 'Color', 'white', 'Position', [100, 100, 800, 500]);
            [sortedImp, sortIdx] = sort(imp, 'ascend');
            sortedNames = featureNames(sortIdx);

            barh(sortedImp, 'FaceColor', [0.2, 0.5, 0.7]);
            yticks(1:length(sortedNames));
            yticklabels(sortedNames);
            xlabel('Out-of-Bag Permuted Feature Importance');
            title('Random Forest Feature Importance Ranking');
            grid on;

            % Ensure directory exists before saving
            configDir = fileparts(modelPath);
            if ~isfolder(configDir), mkdir(configDir); end
            save(modelPath, 'model');
            fprintf('  -> Model successfully saved to: %s\n', modelPath);
        end
    else
        model = modelInput;
    end
    
    % --- STEP 2: FULL-SCALE FEATURE ENGINEERING (Entire Dataset at Full Density) ---
    fprintf('  -> Extracting radius-based geometric features for full dataset (Radius = %.2f m)...\n', config.searchRadius);
    [X_features] = extractRobustFeatures(ptCloudClean);%, config.searchRadius);
    
    % --- STEP 3: CLASSIFICATION INFERENCE ---
    fprintf('  -> Executing Random Forest ensemble prediction on full dataset...\n');
    predictedLabels = predict(model, X_features);
    isRoad = (predictedLabels == true);
    ptCloudRoad = select(ptCloudClean, isRoad);
    
    elapsedTime = toc;
    fprintf('  -> Parallel ML filtering completed in %.2f seconds. Road points extracted: %d\n', ...
        elapsedTime, ptCloudRoad.Count);
    fprintf('============================================\n');
end

%% =========================================================================
% SUB-FUNCTION 1: MODEL TRAINING (Parallelized Ensemble + Opt Flag)
% =========================================================================
function [model, featureNames] = trainRoadClassifier(ptCloudClean, config)
    fprintf('--- STARTING TRAINING (Safe Mode) ---\n');
    tTotal = tic;
    
    % --- 0. SANITIZE POINT CLOUD ---
    locs = ptCloudClean.Location;
    validRows = ~any(isnan(locs) | isinf(locs), 2);
    if ~all(validRows)
        fprintf('  -> Warning: Removing %d invalid points (NaN/Inf)...\n', sum(~validRows));
        ptCloudClean = select(ptCloudClean, validRows);
    end

    % 1. Feature Extraction on the FULL Dense Cloud (Preserving Spatial Context)
    tStage = tic;
    [X_features_full , featureNames]= extractRobustFeatures(ptCloudClean);
    fprintf('  [TIME] Feature Extraction (Full Density): %.2f seconds\n', toc(tStage));
    
    % 2. SMRF Teacher Labeling
    tStage = tic;
    fprintf('  -> Generating automated teacher labels using SMRF...\n');
    ptCloudRoadOnly = local_extractGeometricRoad_forTraining(ptCloudClean,config);
    fprintf('  [TIME] SMRF Teacher Execution: %.2f seconds\n', toc(tStage));
    
    % 3. Label Mapping
    tStage = tic;
    numPoints = ptCloudClean.Count;
    locations = double(ptCloudClean.Location);
    Y_labels_full = false(numPoints, 1);
    if ptCloudRoadOnly.Count > 0
        roadLocations = double(ptCloudRoadOnly.Location);
        [nearestIdx, dist] = knnsearch(locations, roadLocations);
        validMatch = dist < 1e-2; 
        Y_labels_full(nearestIdx(validMatch)) = true;
    end
    fprintf('  [TIME] KNN Label Mapping: %.2f seconds\n', toc(tStage));

    % 4. Row Subsampling for Fast Bayesian Optimization (Without modifying point density)
    maxTrainingPoints = config.minTrainPts; 
    if numPoints > maxTrainingPoints
        fprintf('  -> Subsampling %d rows from feature matrix for rapid Bayesian Optimization...\n', maxTrainingPoints);
        rng(42); % Reproducible sampling seed
        trainIndices = randperm(numPoints, maxTrainingPoints);
        X_features = X_features_full(trainIndices, :);
        Y_labels = Y_labels_full(trainIndices);
    else
        X_features = X_features_full;
        Y_labels = Y_labels_full;
    end

    % 5. Training Logic (Conditional Bayesian Optimization vs Fast Defaults)
    tStage = tic;
    if config.doBayesOpt
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
                    hyperparams(i).Range = [10, 500];
                    hyperparams(i).Optimize = true;
                case 'NumLearningCycles'
                    hyperparams(i).Range = [20, 500];   
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
                'UseParallel', true, ...
                'Verbose', 1));
    else
        fprintf('  -> Skipping Bayesian Optimization. Training with optimized fast defaults...\n');
        model = fitcensemble(X_features, Y_labels, ...
            'Method', 'Bag', ...
            'NumLearningCycles', 30, ...
            'MinLeafSize', 5, ...
            'NumBins', 50, ...
            'Options', statset('UseParallel', true));
    end
    
    fprintf('  [TIME] Model Training Phase: %.2f seconds\n', toc(tStage));
    fprintf('--- TOTAL TRAINING TIME: %.2f seconds ---\n', toc(tTotal));
    fprintf('  -> Optimal Random Forest model trained successfully!\n');
end




function [X_features, featureNames] = extractRobustFeatures(ptCloudIn, kNeighb, radii)
if nargin < 2, kNeighb = 30; end
if nargin < 3, radii = [0.1, 2.0]; end %min at least 2x voxelsize

xyz = double(ptCloudIn.Location);
numPoints = size(xyz, 1);

fprintf('  -> Extracting Definitive Multi-Scale + Geometric Features (%d radii, K=%d) for %d points...\n', ...
    length(radii), kNeighb, numPoints);

% Build KD-Tree once for all spatial queries
kdtree = KDTreeSearcher(xyz);
z = xyz(:,3);

% --- Calculate Total Feature Dimensions for Preallocation ---
numBase = 3; % HeightAboveGround, Roughness, Gradient, Verticality
numPerRadius = 3; % Density, ZRange, ZDeviation, Planarity, SurfaceRoughness
totalFeatures = numBase + (length(radii) * numPerRadius);

hasIntensity = ~isempty(ptCloudIn.Intensity);
hasColor = ~isempty(ptCloudIn.Color);
if hasIntensity, totalFeatures = totalFeatures + 1; end
if hasColor, totalFeatures = totalFeatures + 1; end

% PREALLOCATE memory to prevent OOM and RAM fragmentation
    X_features = zeros(numPoints, totalFeatures, 'single');
    featureNames = cell(1, totalFeatures);
    
    % --- 1. BASE GEOMETRIC FEATURES (K-NN based) ---
    X_features(:, 1) = single(z - min(z));
    featureNames{1} = 'HeightAboveGround';
    
    [idxK1, ~] = knnsearch(kdtree, xyz, 'K', min(kNeighb, numPoints));
    neighborZ = z(idxK1);
    X_features(:, 2) = single(abs(z - mean(neighborZ, 2))); % Roughness
    featureNames{2} = 'Roughness';
    
    % Verticality via Local PCA Normals
    verticality = zeros(numPoints, 1, 'single');
    for i = 1:numPoints
        pts = xyz(idxK1(i, :), :);
        [~, ~, V] = svd(pts - mean(pts, 1), 0); 
        verticality(i) = 1 - abs(V(3, end)); 
    end
    X_features(:, 3) = verticality;
    featureNames{3} = 'Verticality';
    
    colIdx = 3;
    
    % --- 2. MULTI-SCALE FEATURES (Across Radii) ---
    for i = 1:length(radii)
        r = radii(i);
        idxRadius = rangesearch(kdtree, xyz, r);
        
        locDens    = zeros(numPoints, 1, 'single');
        planarity  = zeros(numPoints, 1, 'single');
        surfRough  = zeros(numPoints, 1, 'single');
        
        % Note: If you have Parallel Computing Toolbox, you can change 'for' to 'parfor' here
       for j = 1:numPoints
            ids = idxRadius{j};
            locDens(j) = numel(ids);
            if ~isempty(ids)
                if length(ids) > 4
                    neighborPts = xyz(ids, :);
                    covMat = cov(neighborPts);
                    [~, latent] = eig(covMat);
                    latent = diag(latent); % [l1, l2, l3] ascending
                    l1 = max(latent(3), 1e-6); 
                    l2 = max(latent(2), 1e-6);
                    l3 = max(latent(1), 1e-6);
                    planarity(j) = (l2 - l3) / l1;
                    surfRough(j) = l3 / (l1 + l2 + l3);
                end
            end
        end
        
        % Assign directly to preallocated columns
        X_features(:, colIdx + 1) = locDens;
        X_features(:, colIdx + 2) = planarity;
        X_features(:, colIdx + 3) = surfRough;
        
        % Assign names correctly (Fixed mismatch bug)
        rStr = strrep(sprintf('%.2f', r), '.', '_');
        featureNames{colIdx + 1} = ['LocalDensity_R', rStr];
        featureNames{colIdx + 2} = ['Planarity_R', rStr];
        featureNames{colIdx + 3} = ['SurfaceRoughness_R', rStr];
        
        colIdx = colIdx + 3;
    end
    
    % --- 3. OPTIONAL SENSOR CHANNELS ---
    if hasIntensity
        colIdx = colIdx + 1;
        X_features(:, colIdx) = single(ptCloudIn.Intensity);
        featureNames{colIdx} = 'Intensity';
    end
    
    if hasColor
        colIdx = colIdx + 1;
        rgb = double(ptCloudIn.Color);
        exg = (2 * rgb(:,2) - rgb(:,1) - rgb(:,3)) ./ (sum(rgb, 2) + 1e-5);
        X_features(:, colIdx) = single(exg);
        featureNames{colIdx} = 'ExcessGreen';
    end
    
    fprintf('  -> Feature extraction complete. Total feature dimensions: %d\n', size(X_features, 2));
    
    % --- 4. FEATURE CORRELATION MATRIX & HEATMAP ---
    fprintf('  -> Computing feature correlation matrix...\n');
    corrMat = corr(double(X_features), 'Rows', 'pairwise');
    
    figure('Name', 'Feature Correlation Heatmap', 'Color', 'white', 'Position', [100, 100, 900, 700]);
    heatmap(featureNames, featureNames, corrMat, ...
        'Colormap', turbo, ...
        'ColorLimits', [-1, 1]);
    title('Feature Correlation Matrix (Pruning Guide)');
end
