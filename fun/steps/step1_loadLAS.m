function [ptCloudRaw, wkt, ascFile, prjFile, lasFile] = step1_loadLAS(config,varargin)
    % STEP1_LOADLAS Loads a LAS/LAZ file based on data type flag or manual selection.
    %
    % Usage:
    %   [ptCloudRaw, wkt, ascFile, prjFile] = step1_loadLAS('DataType', 'Georeferenced');
    %   [ptCloudRaw, wkt, ascFile, prjFile] = step1_loadLAS('ManualSelect', true);
    
    p = inputParser;
    addParameter(p, 'DataType', 'Georeferenced', @(x) ismember(x, {'Georeferenced', 'Reconstructed'}));
    addParameter(p, 'ManualSelect', false, @islogical);
    parse(p, varargin{:});
    funConfig = p.Results;
    
    resultsDir = config.defaultLasResults;
    lasFile = '';
    
    % Option 1: Manual Selection
    if funConfig.ManualSelect
        disp('  -> [MANUAL SELECT] Opening file selection dialog...');
        [fileName, fileDir] = uigetfile({'*.las;*.laz', 'LAS / LAZ Files (*.las, *.laz)'}, ...
            'Select LAS/LAZ file', resultsDir);
        if isequal(fileName, 0)
            error('No file selected by user.');
        end
        lasFile = fullfile(fileDir, fileName);
    
    % Option 2: Automatic Selection (DataType)
    else
        if ~isfolder(resultsDir)
            error('  -> [ERROR] Results directory not found: %s. Use ManualSelect = true.', resultsDir);
        end
    
        % Combine both .las and .laz searches to ensure we get the absolute newest
        if strcmp(funConfig.DataType, 'Georeferenced')
            patLas = fullfile(resultsDir, '*_Georeferenced.las');
            patLaz = fullfile(resultsDir, '*_Georeferenced.laz');
        elseif strcmp(funConfig.DataType, 'Reconstructed')
            patLas = fullfile(resultsDir, 'Reconstructed_Master_*.las');
            patLaz = fullfile(resultsDir, 'Reconstructed_Master_*.laz');
        end
    
        files = [dir(patLas); dir(patLaz)];
    
        if isempty(files)
            error('  -> [ERROR] No files matching DataType "%s" found in %s. Try ManualSelect = true.', ...
                funConfig.DataType, resultsDir);
        end
    
        % Pick the most recent file across BOTH extensions
        [~, idx] = max([files.datenum]);
        lasFile = fullfile(files(idx).folder, files(idx).name);
        fprintf('  -> [AUTO SELECT] Selected most recent %s file:\n    %s\n', funConfig.DataType, lasFile);
    end
    
    % Extract paths for auxiliary files
    [fileDir, baseName, ~] = fileparts(lasFile);
    if isempty(fileDir), fileDir = pwd; end
    ascFile = fullfile(fileDir, [baseName '.asc']);
    prjFile = fullfile(fileDir, [baseName '.prj']);
    
    % Read Point Cloud
    fprintf('  -> [LOADING] Reading point cloud file...\n');
    reader = lasFileReader(lasFile);
    wkt = '';
    try
        vlrRecord = reader.readVLR(2112);
        if ~isempty(vlrRecord) && ~isempty(vlrRecord.Data)
            wkt = vlrRecord.Data;
            fprintf('  -> CRS WKT successfully found in VLR 2112.\n');
        end
    catch
        warning('  -> [CRS NOTICE] File does not contain VLR 2112 projection metadata.');
    end

    ptCloudRaw = readPointCloud(reader);
    fprintf('  -> Initial points loaded: %d\n', ptCloudRaw.Count);
    
    % Extract WKT
    fprintf('  -> Applying 5mm micro-voxel deduplication...\n');
    ptCloudRaw = pcdownsample(ptCloudRaw, 'gridAverage', config.minVoxelSize);
    fprintf('  -> Points retained after deduplication: %d\n', ptCloudRaw.Count);
    
    % --- STAGE 1 VISUALIZATION (Optional) ---
    if isfield(config, 'visible') && config.visible
        fprintf('  -> [VISUALIZATION] Rendering Stage 1 point cloud...\n');
        figure('Name', 'Stage 1: Raw Point Cloud (Deduplicated)', 'Color', 'w');
        pcshow(ptCloudRaw);
        title(sprintf('Stage 1: %s (%d points)', baseName, ptCloudRaw.Count), 'Interpreter', 'none');
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        drawnow;
    end
end