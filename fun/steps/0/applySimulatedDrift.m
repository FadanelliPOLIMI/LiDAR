function ptCloudOut = applySimulatedDrift(ptCloudIn, config)
    % APPLYSIMULATEDDRIFT Injects advanced EW/GPS artifacts:
    % 1. Non-uniform dynamic spatial wobble (trajectory undulation)
    % 2. Unbiased sensor jitter
    % 3. Sparse multipath ghost points (outlier spikes)
    
    if ~isfield(config, 'simulateDrift') || ~config.simulateDrift
        ptCloudOut = ptCloudIn;
        return;
    end
    
    disp(' >>> [EW SIMULATION] Injecting advanced EW/GPS distortion artifacts...');
    originalPts = ptCloudIn.Location;
    numPts = size(originalPts, 1);
    
    % Safe config field retrieval (supports both lowercase and PascalCase)
    bEast  = getCfg(config, {'BiasEast', 'biasEast'}, 3.5);
    bNorth = getCfg(config, {'BiasNorth', 'biasNorth'}, -2.2);
    bUp    = getCfg(config, {'BiasUp', 'biasUp'}, 0.15);
    stdDev = getCfg(config, {'UnbiasedStd', 'unbiasedStd'}, 0.05);
    
    noisyPts = originalPts;
    
    % =====================================================================
    % 1. NON-UNIFORM DYNAMIC DRIFT (Spatial Wobble)
    % =====================================================================
    wobbleX = sin(originalPts(:,2) * 0.5) * 0.8; % Spatial undulation
    wobbleY = cos(originalPts(:,1) * 0.5) * 0.8;
    
    noisyPts(:, 1) = originalPts(:, 1) + bEast + wobbleX;
    noisyPts(:, 2) = originalPts(:, 2) + bNorth + wobbleY;
    noisyPts(:, 3) = originalPts(:, 3) + bUp;
    
    % =====================================================================
    % 2. STANDARD SENSOR JITTER
    % =====================================================================
    noisyPts = noisyPts + randn(numPts, 3) * stdDev;
    
    % =====================================================================
    % 3. INJECT EW MULTIPATH / GHOST POINTS (2% wild outliers)
    % =====================================================================
    numGhosts = round(numPts * 0.02);
    if numGhosts > 0
        ghostIndices = randperm(numPts, numGhosts);
        noisyPts(ghostIndices, 3) = noisyPts(ghostIndices, 3) + (rand(numGhosts, 1) - 0.5) * 15.0; 
        noisyPts(ghostIndices, 1) = noisyPts(ghostIndices, 1) + (rand(numGhosts, 1) - 0.5) * 10.0;
    end
    
    % Retain color and intensity attributes if present
    if isprop(ptCloudIn, 'Color') && ~isempty(ptCloudIn.Color)
        cols = ptCloudIn.Color;
    else
        cols = uint8(ones(numPts, 3) * 128);
    end
    
    if isprop(ptCloudIn, 'Intensity') && ~isempty(ptCloudIn.Intensity)
        ints = ptCloudIn.Intensity;
        ptCloudOut = pointCloud(noisyPts, 'Color', cols, 'Intensity', ints);
    else
        ptCloudOut = pointCloud(noisyPts, 'Color', cols);
    end
    
    fprintf('     - Advanced EW Simulation Applied:\n');
    fprintf('       * Biased Shift: East=%.2fm, North=%.2fm, Up=%.2fm\n', bEast, bNorth, bUp);
    fprintf('       * Non-uniform wobble & Jitter Std=%.2fm\n', stdDev);
    fprintf('       * Injected Multipath Ghosts: %d points (2%%)\n', numGhosts);
end

% Helper function for robust config field extraction
function val = getCfg(cfg, names, defaultVal)
    val = defaultVal;
    for i = 1:length(names)
        if isfield(cfg, names{i})
            val = cfg.(names{i});
            return;
        end
    end
end


function ptCloudOut = applySimulatedDrift(ptCloudIn, config)
% APPLYSIMULATEDDRIFT Injects advanced EW/GPS artifacts:
% 1. Non-uniform dynamic spatial wobble (trajectory undulation)
% 2. Unbiased sensor jitter
% 3. Sparse multipath ghost points (outlier spikes)

if ~isfield(config, 'simulateDrift') || ~config.simulateDrift
    ptCloudOut = ptCloudIn;
    return;
end

disp(' >>> [EW SIMULATION] Injecting advanced EW/GPS distortion artifacts...');
originalPts = ptCloudIn.Location;
numPts = size(originalPts, 1);

% Safe config field retrieval (supports both lowercase and PascalCase)
bEast  = getCfg(config, {'BiasEast', 'biasEast'}, 3.5);
bNorth = getCfg(config, {'BiasNorth', 'biasNorth'}, -2.2);
bUp    = getCfg(config, {'BiasUp', 'biasUp'}, 0.15);
stdDev = getCfg(config, {'UnbiasedStd', 'unbiasedStd'}, 0.05);

noisyPts = originalPts;

% =====================================================================
% 1. NON-UNIFORM DYNAMIC DRIFT (Spatial Wobble)
% =====================================================================
wobbleX = sin(originalPts(:,2) * 0.5) * 0.8; % Spatial undulation
wobbleY = cos(originalPts(:,1) * 0.5) * 0.8;

noisyPts(:, 1) = originalPts(:, 1) + bEast + wobbleX;
noisyPts(:, 2) = originalPts(:, 2) + bNorth + wobbleY;
noisyPts(:, 3) = originalPts(:, 3) + bUp;

% =====================================================================
% 2. STANDARD SENSOR JITTER
% =====================================================================
noisyPts = noisyPts + randn(numPts, 3) * stdDev;

% =====================================================================
% 3. INJECT EW MULTIPATH / GHOST POINTS (2% wild outliers)
% =====================================================================
numGhosts = round(numPts * 0.02);
if numGhosts > 0
    ghostIndices = randperm(numPts, numGhosts);
    noisyPts(ghostIndices, 3) = noisyPts(ghostIndices, 3) + (rand(numGhosts, 1) - 0.5) * 15.0; 
    noisyPts(ghostIndices, 1) = noisyPts(ghostIndices, 1) + (rand(numGhosts, 1) - 0.5) * 10.0;
end

% Retain color and intensity attributes if present
if isprop(ptCloudIn, 'Color') && ~isempty(ptCloudIn.Color)
    cols = ptCloudIn.Color;
else
    cols = uint8(ones(numPts, 3) * 128);
end

if isprop(ptCloudIn, 'Intensity') && ~isempty(ptCloudIn.Intensity)
    ints = ptCloudIn.Intensity;
    ptCloudOut = pointCloud(noisyPts, 'Color', cols, 'Intensity', ints);
else
    ptCloudOut = pointCloud(noisyPts, 'Color', cols);
end

fprintf('     - Advanced EW Simulation Applied:\n');
fprintf('       * Biased Shift: East=%.2fm, North=%.2fm, Up=%.2fm\n', bEast, bNorth, bUp);
fprintf('       * Non-uniform wobble & Jitter Std=%.2fm\n', stdDev);
fprintf('       * Injected Multipath Ghosts: %d points (2%%)\n', numGhosts);
end

% Helper function for robust config field extraction
function val = getCfg(cfg, names, defaultVal)
val = defaultVal;
for i = 1:length(names)
    if isfield(cfg, names{i})
        val = cfg.(names{i});
        return;
    end
end
end