function config = initPipelineConfig(varargin)
    % INITPIPELINECONFIG Initializes and returns configuration structure.
    %
    % Ex:
    %   config = initPipelineConfig();                          % Uses default
    %   config = initPipelineConfig('forceAll', true);          % Forces all boolean flags to true
    %   config = initPipelineConfig('alignmentMode', 'auto');   % automatic DL Aligment 
    
    p = inputParser;
    addParameter(p, 'forceAll', [], @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'alignmentMode', 'manual', @ischar);
    addParameter(p, 'visible', false, @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});
    opts = p.Results;
    
    % =====================================================================
    % GLOBAL & PATH PARAMETERS
    % =====================================================================
    config.mode             = 'laptop'; % 'laptop' or 'edge'
    config.alignmentMode    = opts.alignmentMode;
    config.visible          = opts.visible;
    config.defaultLasDir    = fullfile('files', 'las'); 
    config.defaultLasResults= fullfile('files', 'las', 'results'); 
    config.defaultConfigs   = fullfile('files', 'configs'); 

    % =====================================================================
    % STAGE 1: LOADING & DEDUPLICATION PARAMETERS
    % =====================================================================
    config.minVoxelSize     = 0.005;  % [m] 5mm micro-voxel deduplication

    % =====================================================================
    % STAGE 2: PREPROCESSING & QUALITY GATE PARAMETERS
    % =====================================================================
    if strcmpi(config.mode, 'edge')
        config.sorNeighbors         = 20;
        config.sorThreshold         = 1.5;
        config.voxelSize            = 0.08;
        config.rmseThreshold        = 0.035;
        config.maxErrorThreshold    = 0.050;
        config.kNeighbors           = 15;
        config.verticalityThreshold = 0.55;
    else % Laptop / Default
        config.sorNeighbors         = 30;
        config.sorThreshold         = 1.0;
        config.voxelSize            = 0.05;
        config.rmseThreshold        = 0.015;
        config.maxErrorThreshold    = 0.020;
        config.kNeighbors           = 30;
        config.verticalityThreshold = 0.55;
    end

    % =====================================================================
    % STAGE 3: EXTRACTION & ROI PARAMETERS
    % =====================================================================
    config.extractionMode   = 'interactive'; % Options: 'auto', 'interactive', 'preset'
    config.roiType          = 'rectangle';   % Options: 'rectangle', 'polygon'
    config.roiPolygon       = [];            % Stores polygon vertices [X, Y]

    % =====================================================================
    % STAGE 4: CLASSIFICATION & MODELING EXPORT PARAMETERS
    % =====================================================================
    config.classMode        = 'manual';        % 'auto' or 'manual'
    config.resPercent       = 0.25;          % [%] Percentage of minor side of AOI
    config.targetPtsPerCell = 4;             % Desired average LAS points per cell
    config.cellSizeQuantum  = 0.001;         % Final cell quantization [m] (1 mm)
    config.nodata           = -9999;         % Standard value for empty cells
    config.sliceWidth       = 0.5;           % Slice width for hole filling
    config.MinHeightLimit   = 0.10;          % [m] Minimum height limit for georef pipeline
    config.maxBandwidth     = [];            % Maximum Transmission BW
    
    % =====================================================================
    % STAGE 8: ROAD EXPORT PARAMETERS
    % =====================================================================
    config.maxSafeGradientPct = 15.0;
    
    % =====================================================================
    % PIPELINE CONTROL & SIMULATION FLAGS
    % =====================================================================
    config.enableGeoref     = false;   % LiDAR-satellite data fusion
    config.compareFilters   = false;   
    config.optimizeFilter   = false;  
    config.compareInterp    = false;  
    config.optimizeInterp   = false;  
    config.fillingHoles     = false;   
    config.gaussFilt        = false;  
    config.simulateDrift    = false;   % GPS error simulation

    % GPS Drift Simulation Testing Parameters
    config.biasEast         = 3.50;   % [m] Artificial Easting bias
    config.biasNorth        = -2.20;  % [m] Artificial Northing bias
    config.biasUp           = 0.15;   % [m] Artificial Elevation bias
    config.unbiasedStd      = 0.05;   % [m] Standard deviation of random sensor jitter

    % =====================================================================
    % DYNAMIC OVERRIDE LOGIC
    % =====================================================================
    if ~isempty(opts.forceAll)
        forcedVal = logical(opts.forceAll);
    
        controlFlags = {'enableGeoref', 'compareFilters', 'optimizeFilter', ...
            'compareInterp', 'optimizeInterp', 'fillingHoles', ...
            'gaussFilt', 'simulateDrift'};
    
        for i = 1:length(controlFlags)
            config.(controlFlags{i}) = forcedVal;
        end
        fprintf('  -> [Override Active] All boolean flags dynamically forced to: %d\n', forcedVal);
    else
        fprintf('  -> [Custom Mode] Using individual settings (Alignment: %s).\n', config.alignmentMode);
    end
end