function [ptCloudRoad, winningMethod, benchmarkMetrics, bestHeuristicParams, rfModel] = ...
    step4_classFilters(ptCloudExtracted, ptCloudAligned, ptCloudFiltered, config)
% STEP4_CLASSFILTERS - Stage 4 Ground/Road Classification & Benchmarking Wrapper

if nargin < 1 || isempty(ptCloudExtracted)
    error('PipelineError:MissingInput', 'ptCloudExtracted input is required for Stage 4.');
end
if nargin < 2 || isempty(ptCloudAligned), ptCloudAligned = ptCloudExtracted; end
if nargin < 3 || isempty(ptCloudFiltered), ptCloudFiltered = ptCloudAligned; end
if nargin < 4 || isempty(config), config = initPipelineConfig(); end
if ~isfield(config, 'mode'), config.mode = 'laptop'; end
ticTotal = tic;

isPcaFailed = false;
if isfield(config, 'geoTransform') && isfield(config.geoTransform, 'pcaFailed')
    isPcaFailed = config.geoTransform.pcaFailed;
end

fprintf('\n------------------------------------------------------------\n');
fprintf('>>> STAGE 4: ROAD GROUND CLASSIFICATION & BENCHMARKING\n');
fprintf('------------------------------------------------------------\n');
fprintf('  -> Target Pipeline Architecture:\n');
fprintf('     1. Training/Tuning Sandbox : ptCloudExtracted (%d pts)\n', ptCloudExtracted.Count);
fprintf('     2. Inference Target (RF/Heur): ptCloudAligned (%d pts)\n', ptCloudAligned.Count);
fprintf('     3. Inference Target (DL)   : ptCloudFiltered (%d pts)\n', ptCloudFiltered.Count);

% --- STEP 4A: MASTER REFERENCE TEACHER (Strictly on ROI) ---
fprintf('\n  -> [STEP 4A] Generating Master Ground Teacher on Extracted ROI Cloud...\n');
ptCloudMaster = extractGeometricRoad(ptCloudExtracted, config, isPcaFailed);

% --- STEP 4B: OPTIMIZATION & TRAINING (Strictly on ROI) ---
if strcmpi(config.mode, 'edge')
    fprintf('  -> [EDGE MODE] Bypassing live Bayesian Search & RF Training.\n');
    [bestHeuristicParams, rfModel] = loadCachedOrDefaultParams(config);
else
    fprintf('  -> [LAPTOP MODE] Running Bayesian Optimization & RF Training on Extracted ROI...\n');
    [bestHeuristicParams, rfModel] = bayesOptFilt(ptCloudMaster, ptCloudExtracted, config);
end

% --- STEP 4C: MULTI-MODEL BENCHMARKING ---
fprintf('\n  -> [STEP 4C] Benchmarking candidate classification algorithms...\n');
[ptCloudRoad, winningMethod, benchmarkMetrics] = classificationFilter(...
    ptCloudAligned, ptCloudFiltered, ptCloudMaster, config, bestHeuristicParams, rfModel);

benchmarkMetrics.totalStage4Time = toc(ticTotal);

% Find denominator for final printout based on winning method
if strcmpi(winningMethod, 'Deep Learning')
    baseCount = ptCloudFiltered.Count;
else
    baseCount = ptCloudAligned.Count;
end

fprintf('\n  -> [SUCCESS] Stage 4 complete. Winning Classification Method: %s\n', winningMethod);
fprintf('  -> Final Road Points Extracted: %d / %d (%.1f%% of its inference cloud)\n', ...
    ptCloudRoad.Count, baseCount, (ptCloudRoad.Count / max(1, baseCount)) * 100);
fprintf('------------------------------------------------------------\n');
end

function [bestHeuristicParams, rfModel] = loadCachedOrDefaultParams(config)
cacheFile = fullfile(config.defaultConfigs, 'heuristic_rf_cache.mat');
if exist(cacheFile, 'file')
    loaded = load(cacheFile);
    bestHeuristicParams = loaded.trained_heuristic;
    rfModel = loaded.trained_rf;
    fprintf('  -> Loaded pre-tuned parameters from cache: %s\n', cacheFile);
else
    bestHeuristicParams = struct('cellResolution', single(0.15), 'maxGroundSlope', single(0.05), ...
        'elevationThreshold', single(0.08), 'windowSize', single(15));
    rfModel = []; 
    fprintf('  -> Cache not found. Using default robust single-precision heuristic parameters.\n');
end
end