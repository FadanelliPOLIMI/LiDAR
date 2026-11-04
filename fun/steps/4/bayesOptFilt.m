function [bestHeuristicParams, rfModel] = bayesOptFilt(ptCloudMaster, ptCloudWork, config)
    % BAYESOPTFILT - Streamlined cache manager and optimizer for Stage 4.
    % Automatically handles edge mode, missing cache fallback, and Bayesian tuning.
    % Standardized cache variables: trained_heuristic & trained_rf.
    
    if nargin < 3 || isempty(config), config = initPipelineConfig(); end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end
    if ~isfield(config, 'defaultConfigs'), config.defaultConfigs = fullfile('files', 'configs'); end
    
    if ~isfolder(config.defaultConfigs), mkdir(config.defaultConfigs); end
    
    cacheFile = fullfile(config.defaultConfigs, 'heuristic_rf_cache.mat');
    
    % --- 1. EDGE MODE FALLBACK ---
    if strcmpi(config.mode, 'edge')
        if exist(cacheFile, 'file')
            loaded = load(cacheFile);
            bestHeuristicParams = loaded.trained_heuristic;
            rfModel = loaded.trained_rf;
            fprintf('  -> [EDGE] Loaded pre-tuned parameters and model from cache.\n');
        else
            % Single-precision default fallback for Jetson Nano
            bestHeuristicParams = struct(...
                'cellResolution',     single(0.15), ...
                'maxGroundSlope',     single(0.05), ...
                'elevationThreshold', single(0.08), ...
                'windowSize',         single(15));
            rfModel = []; % Empty model on Edge fallback
            fprintf('  -> [EDGE] Cache not found. Using default single-precision parameters.\n');
        end
        return;
    end
    
    % --- 2. SMART CACHE & OPTIMIZATION CHECK ---
    forceOptimize = isfield(config, 'optimizeFilter') && config.optimizeFilter;
    cacheExists   = exist(cacheFile, 'file');
    
    if cacheExists && ~forceOptimize
        fprintf('  -> Loading pre-saved parameters and RF model from cache: %s\n', cacheFile);
        loaded = load(cacheFile);
        bestHeuristicParams = loaded.trained_heuristic;
        rfModel = loaded.trained_rf;
        return;
    end
    
    % --- 3. RUN BAYESIAN OPTIMIZATION & TRAINING (Cache missing or forced) ---
    if ~cacheExists
        fprintf('  -> Cache file not found. Forcing parameter optimization and training...\n');
    else
        fprintf('  -> Force optimization requested. Re-running Bayesian Optimization...\n');
    end
    
    fprintf('\n------------------------------------------------------------\n');
    fprintf('>>> Running Bayesian Optimization for Heuristic Filter...\n');
    fprintf('------------------------------------------------------------\n');
    
    heuristicVariables = [
        optimizableVariable('binSize', [0.05, 0.3], 'Transform', 'log')
        optimizableVariable('macroSize', [35.0, 100.0])    
        optimizableVariable('minSearchHeight', [0.05, 0.25])
        optimizableVariable('K1', [20, 70], 'Type', 'integer')
        optimizableVariable('K2', [30, 80], 'Type', 'integer')
        optimizableVariable('maxNumSplits', [20, 50], 'Type', 'integer')
        optimizableVariable('greenThreshold', [0.15, 0.90]) 
        ];
    
    % Safe Parallel Pool Check (prevents crash if Parallel Toolbox is missing)
    useParallelPool = ~isempty(ver('parallel')) && ~isempty(gcp('nocreate'));
    
    resultsHeuristic = bayesopt(@(p) costFunctionHST(p, ptCloudWork), heuristicVariables, ...
        'MaxObjectiveEvaluations', 20, 'UseParallel', useParallelPool, 'Verbose', 0);
    
    trained_heuristic = bestPoint(resultsHeuristic);
    bestHeuristicParams = trained_heuristic;
    
    % Optional Headless-Safe Plotting
    if isfield(config, 'visible') && config.visible && ~strcmpi(config.mode, 'edge')
        figure('Name', 'Bayesian Optimization Progress', 'Color', 'w');
        plot(resultsHeuristic);
        title('Heuristic Filter Parameter Optimization History', 'Color', 'k');
        drawnow;
    end
    
    fprintf('\n------------------------------------------------------------\n');
    fprintf('>>> Training / Optimizing Random Forest Model...\n');
    fprintf('------------------------------------------------------------\n');
    
    config.optimizeFilter = true; 
    config.minTrainPts = 1000;
    
    [~, trained_rf] = randomForest(ptCloudMaster, ptCloudWork, config);
    rfModel = trained_rf;
    
    % Save unified cache using standardized names
    save(cacheFile, 'trained_heuristic', 'trained_rf');
    fprintf('  -> Successfully saved optimized parameters and RF model to cache: %s\n', cacheFile);
end