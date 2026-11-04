function [ptCloudRoad, selectedMethodName, benchmarkMetrics] = classificationFilter(...
    ptCloudAligned, ptCloudFiltered, ptCloudMaster, config, bestParameters, rfModel)
    % CLASSIFICATIONFILTER - Hardware-aware Benchmarking with Context Separation
    
    fprintf('\n--------------------------------------------\n');
    fprintf('AUTOMATED STAGE: BENCHMARK & AUTO-SELECTOR\n');
    fprintf('--------------------------------------------\n');
    
    if nargin < 1 || isempty(ptCloudAligned), error('ptCloudAligned is required.'); end
    if nargin < 2 || isempty(ptCloudFiltered), ptCloudFiltered = ptCloudAligned; end
    if nargin < 3 || isempty(ptCloudMaster), ptCloudMaster = ptCloudAligned; end
    if nargin < 4 || isempty(config), config = initPipelineConfig(); end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end
    if nargin < 5, bestParameters = struct(); end
    if nargin < 6, rfModel = []; end
    
    methodsList = {'Random Forest', 'Deep Learning', 'Heuristic'};
    numMethods = length(methodsList);
    extractedClouds = cell(1, numMethods);
    executionTimes  = zeros(1, numMethods);
    pointCounts     = zeros(1, numMethods);
    zMADs           = zeros(1, numMethods);
    
    % Store the base input count used by each method to prevent impossible >100% math
    inputCounts = [ptCloudAligned.Count, ptCloudFiltered.Count, ptCloudAligned.Count];
    
    % --- 1. Random Forest (3A) - INFERENCES ON ptCloudAligned ---
    fprintf('\n--- [1/3] Executing Random Forest ---\n');
    tic;
    try
        [extractedClouds{1}, ~] = randomForest(ptCloudMaster, ptCloudAligned, config, rfModel);
    catch ME
        warning('RF Failed: %s. Using empty fallback.', ME.message);
        extractedClouds{1} = pointCloud([0 0 0]);
    end
    executionTimes(1) = toc;
    
    % --- 2. Deep Learning RandLA-Net (3B) - INFERENCES ON ptCloudFiltered ---
    fprintf('\n--- [2/3] Executing Deep Learning RandLA-Net ---\n');
    tic;
    try
        extractedClouds{2} = DLRandLANet(ptCloudFiltered, config);
    catch ME
        warning('DL Skipped/Failed: %s. Using empty fallback.', ME.message);
        extractedClouds{2} = pointCloud([0 0 0]);
    end
    executionTimes(2) = toc;
    
    % --- 3. Active Learning Heuristic (3C) - INFERENCES ON ptCloudAligned ---
    fprintf('\n--- [3/3] Executing Heuristic Sampling ---\n');
    tic;
    try
        bSize = []; mSize = []; mRef = []; pMode = []; minH = []; 
        k1Val = []; k2Val = []; maxSplits = []; greenThresh = [];
        if ~isempty(bestParameters)
            if istable(bestParameters)
                varNames = bestParameters.Properties.VariableNames;
                if any(strcmp(varNames, 'binSize')), bSize = bestParameters.binSize(1); end
                if any(strcmp(varNames, 'macroSize')), mSize = bestParameters.macroSize(1); end
                if any(strcmp(varNames, 'minSearchHeight')), minH = bestParameters.minSearchHeight(1); end
                if any(strcmp(varNames, 'K1')), k1Val = bestParameters.K1(1); end
                if any(strcmp(varNames, 'K2')), k2Val = bestParameters.K2(1); end
                if any(strcmp(varNames, 'maxNumSplits')), maxSplits = bestParameters.maxNumSplits(1); end
                if any(strcmp(varNames, 'greenThreshold')), greenThresh = bestParameters.greenThreshold(1); end
            elseif isstruct(bestParameters)
                if isfield(bestParameters, 'binSize'), bSize = bestParameters.binSize; end
                if isfield(bestParameters, 'macroSize'), mSize = bestParameters.macroSize; end
                if isfield(bestParameters, 'minSearchHeight'), minH = bestParameters.minSearchHeight; end
                if isfield(bestParameters, 'K1'), k1Val = bestParameters.K1; end
                if isfield(bestParameters, 'K2'), k2Val = bestParameters.K2; end
                if isfield(bestParameters, 'maxNumSplits'), maxSplits = bestParameters.maxNumSplits; end
                if isfield(bestParameters, 'greenThreshold'), greenThresh = bestParameters.greenThreshold; end
            end
        end
        [extractedClouds{3}, ~, ~, ~] = heuristicSelfTrain(ptCloudAligned, mRef, bSize, mSize, pMode, minH, k1Val, k2Val, maxSplits, greenThresh, false);
    catch ME
        warning('Heuristic Sampling Failed: %s. Using empty fallback.', ME.message);
        extractedClouds{3} = pointCloud([0 0 0]);
    end
    executionTimes(3) = toc;
    
    % --- Compute Metrics & Composite Scores ---
    scores = zeros(1, numMethods);
    
    for i = 1:numMethods
        ptC = extractedClouds{i};
        pointCounts(i) = ptC.Count;
        
        if ptC.Count > 1
            xyz = double(ptC.Location);
            zMADs(i) = median(abs(xyz(:,3) - median(xyz(:,3))));
        else
            zMADs(i) = inf; 
        end
        
        % DYNAMIC RATIO VALIDITY: Calculates correctly against its own input context!
        retentionRatio = pointCounts(i) / max(1, inputCounts(i));
        
        if retentionRatio >= 0.05 && retentionRatio <= 0.85
            validityBonus = 1.0;
        else
            validityBonus = 0.1; 
        end
        
        if strcmpi(config.mode, 'edge')
            w_smooth = 1.0; w_time = 5.0; 
        else
            w_smooth = 5.0; w_time = 1.0; 
        end
        
        smoothnessScore = 1.0 / (zMADs(i) + 1e-4);
        speedScore      = 1.0 / (executionTimes(i) + 0.1);
        
        scores(i) = (w_smooth * smoothnessScore + w_time * speedScore) * validityBonus;
    end
    
    % --- Automatically Select Winner ---
    [bestScore, bestIdx] = max(scores);
    ptCloudRoad = extractedClouds{bestIdx};
    selectedMethodName = methodsList{bestIdx};
    
    % --- Populate Metrics Output Structure ---
    benchmarkMetrics.methods        = methodsList;
    benchmarkMetrics.executionTimes = executionTimes;
    benchmarkMetrics.pointCounts    = pointCounts;
    benchmarkMetrics.zMADs          = zMADs;
    benchmarkMetrics.scores         = scores;
    benchmarkMetrics.winnerIndex    = bestIdx;
    benchmarkMetrics.winnerName     = selectedMethodName;
    
    % --- Terminal Benchmark Report ---
    fprintf('\n======================== AUTOMATED BENCHMARK & SELECTION RESULTS ========================\n');
    fprintf('METRIC                      | RANDOM FOREST      | DEEP LEARNING      | HEURISTIC SAMPLING \n');
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('Execution Time (sec)        | %6.2f sec        | %6.2f sec        | %6.2f sec        \n', executionTimes(1), executionTimes(2), executionTimes(3));
    fprintf('Extracted Road Points       | %8d           | %8d           | %8d           \n', pointCounts(1), pointCounts(2), pointCounts(3));
    fprintf('True Cloud Retention Rate   | %6.1f%%           | %6.1f%%           | %6.1f%%           \n', (pointCounts(1)/inputCounts(1))*100, (pointCounts(2)/inputCounts(2))*100, (pointCounts(3)/inputCounts(3))*100);
    fprintf('Asphalt Planarity (MAD Z)   | %6.4f m          | %6.4f m          | %6.4f m          \n', zMADs(1), zMADs(2), zMADs(3));
    fprintf('Composite Fitness Score     | %6.2f            | %6.2f            | %6.2f            \n', scores(1), scores(2), scores(3));
    fprintf('========================================================================================\n');
    fprintf(' >>> [AUTO-SELECTOR] Winning Method Selected Automatically: %s (Score: %.2f) <<<\n', selectedMethodName, bestScore);
    
    % --- Comparative 3D Visualization (HEADLESS SAFE) ---
    if isfield(config, 'visible') && config.visible && ~strcmpi(config.mode, 'edge')
        figure('Name', 'Automated Benchmark: RF vs DL vs Active Learning', 'Color', 'w', 'Position', [50, 100, 1500, 600]);
        for i = 1:numMethods
            ax = subplot(1, 3, i);
            if extractedClouds{i}.Count > 1
                xyz_plot = extractedClouds{i}.Location;
                pcshow(xyz_plot, xyz_plot(:,3), 'MarkerSize', 10, 'Parent', ax);
                colormap(ax, jet);
            end
            isWinnerStr = ''; if i == bestIdx, isWinnerStr = ' [WINNER]'; end
            title(sprintf('%s%s\nPts: %d | MAD: %.4fm', methodsList{i}, isWinnerStr, pointCounts(i), zMADs(i)), ...
                'FontSize', 10, 'FontWeight', 'bold');
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            grid on; axis equal; view(3);
        end
        drawnow;
    end
end