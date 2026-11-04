function [allBatchFiles, config] = step0a_runPipeline(config, varargin)
    % RUNPIPELINE Orchestrates the LiDAR ingestion, georeferencing, and tactical batching pipeline.
    %
    % Outputs:
    %   allBatchFiles  - Cell array containing all generated tactical burst file paths.
    %   config - config file
    %
    % Optional Parameters:
    %   'FileName' - String, Cell Array, or 'default' (interactive selection). Default: 'default'.
    %   'Force'    - Boolean (true/false) to force all pipeline config flags. Default: false.

    p = inputParser;
    addParameter(p, 'FileName', 'default', @(x) ischar(x) || isstring(x) || iscell(x));
    addParameter(p, 'Force', false, @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});
    opts = p.Results;

    % =========================================================================
    % 1. GLOBAL CONFIGURATION PARAMETERS & FLAGS
    % =========================================================================
    if opts.Force
        config = initPipelineConfig('forceAll', true);
        disp('  -> [CONFIG] Pipeline initialized with FORCE ALL flags enabled.');
    else
        disp('  -> [CONFIG] Pipeline initialized with default flags.');
    end

    % Ensure results directory is defined in config
    if ~isfield(config, 'defaultLasResults') || isempty(config.defaultLasResults)
        config.defaultLasResults = fullfile('files', 'las', 'results');
    end
    if ~isfolder(config.defaultLasResults)
        mkdir(config.defaultLasResults);
    end

    % =========================================================================
    % 2. CLEAN & STREAMLINED FILE SELECTION
    % =========================================================================
    if ischar(opts.FileName) && strcmpi(opts.FileName, 'default')
        defaultDir = fullfile('files', 'las');
        
        fprintf('  -> [SELECTION] Opening interactive dialog for LAS/LAZ file...\n');
        [file, path] = uigetfile({'*.las;*.laz', 'LAS/LAZ Point Clouds (*.las, *.laz)'}, ...
            'Select LAS/LAZ file', defaultDir);
        
        if isequal(file, 0)
            fprintf('  -> [NOTICE] No file selected. Using default preset...\n');
            fileName = {fullfile(defaultDir, 'Alta Frequenza 5 cm.las')};
        else
            fileName = {fullfile(path, file)};
        end
    else
        fileName = cellstr(opts.FileName);
    end

    fprintf('------------------------------------------------------------\n');
    fprintf('  -> [BATCH] Starting processing of %d file(s)...\n', length(fileName));
    fprintf('------------------------------------------------------------\n');

    allBatchFiles = {};

    % =========================================================================
    % 3. MAIN PROCESSING LOOP
    % =========================================================================
    for i = 1:length(fileName)
        currentFilePath = fileName{i};
        
        fprintf('------------------------------------------------------------\n');
        fprintf('  -> Processing file %d of %d: %s\n', i, length(fileName), currentFilePath);
        fprintf('------------------------------------------------------------\n');

        try
            % -------------------------------------------------------------
            % STAGE 1: INGESTION, INTEGRITY & GEOREFERENCING - OUTPUT LAS
            % DATA GEOREFERENCED AS CLEAN FROM LIDAR DRONE
            % -------------------------------------------------------------
            [ptCloudProcessed, baseName,~,wkt] = ingestAndGeoref(currentFilePath, config);

            % -------------------------------------------------------------
            % STAGE TO DEVELOP: EW AND DATA FROM EW SIMULATION
            % -------------------------------------------------------------
            % %%%%Matlab
            % 
            % % EXAMPLE: Stream-Level Sanity Check at Gate 1
            % function validateStreamTelemetry(metadata)
            %     % Check if GPS quality degraded due to jamming/spoofing
            %     if isfield(metadata, 'FixType') && metadata.FixType < 4 % e.g., 4 = RTK Fixed
            %         warning('>>> [EW ALERT] GPS lost RTK Fixed status! Potential jamming detected.');
            %         % Action: Switch system state to GPS-Denied / IMU-Fallback
            %     end
            % 
            %     if isfield(metadata, 'HDOP') && metadata.HDOP > 5.0
            %         error('>>> [GATE 1 REJECT] High Dilution of Precision (HDOP). Stream rejected due to severe multi-path/EW interference.');
            %     end
            % end
            % 
            % Summary Rule for the Pipeline
            % 
            % At the Gate (Input): Intercept and reject malformed syntax and obvious telemetry/GPS health failures (drop the bad stream instantly).
            % 
            % At Step 2A (Preprocessing): Filter and strip embedded geometric anomalies like ghost points and high-frequency jitter (Statistical Tail-Ratio / SOR).
            % 
            % At the Reconstructor/Backend: Manage unavoidable structural shifts (global bias and trajectory wobble) using spatial alignment and coordinate frames.
            % 
            % Does this schema map cleanly to how you want to handle the incoming data streams on the Jetson Nano edge device?
            
            % -------------------------------------------------------------
            % STAGE 2: TACTICAL CHUNKING & VARIABLE BURST EXPORT --TO BE
            % SUBSTITUTED WITH REAL DATA FROM SENSOR (EW / NOT EW)
            % -------------------------------------------------------------
            batchFiles = tacticalBatching(ptCloudProcessed, baseName, config, wkt);
            
            % Collect outputs
            
            allBatchFiles = [allBatchFiles, batchFiles];

        catch ME
            fprintf('  -> [ERROR] Processing failed for: %s\n', currentFilePath);
            fprintf('  -> Message: %s\n', ME.message);
            if ~isempty(ME.stack)
                fprintf('  -> Location: ''%s'' at line %d\n', ME.stack(1).name, ME.stack(1).line);
            end
            continue; 
        end
    end

    fprintf('------------------------------------------------------------\n');
    disp('  -> [BATCH COMPLETED] All files processed and structured for tactical transmission!');
    fprintf('------------------------------------------------------------\n');
end