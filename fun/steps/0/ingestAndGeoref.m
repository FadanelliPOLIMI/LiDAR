function [ptCloudProcessed, baseName, resolvedFilePath, wkt] = ingestAndGeoref(currentFileName, config)
% INGESTANDGEOREF Resolves file path, validates LAS header, applies georeferencing and GPS drift.

[~, baseName, ~] = fileparts(currentFileName);

% =================================================================
% GATE 1: INGESTION & INTEGRITY CHECK (Upward Hierarchical Search)
% =================================================================
disp(' >>> [GATE] Resolving path upward from base folder to root...');
resolvedFilePath = '';
searchDir = config.defaultLasDir; 

while true
    candidate = fullfile(searchDir, currentFileName);
    if exist(candidate, 'file') == 2
        resolvedFilePath = candidate;
        break;
    end
    [parentDir, ~, ~] = fileparts(searchDir);
    if isempty(parentDir) || strcmp(parentDir, searchDir)
        if exist(currentFileName, 'file') == 2
            resolvedFilePath = currentFileName;
        end
        break;
    end
    searchDir = parentDir;
end

if isempty(resolvedFilePath)
    error('PipelineError:FileNotFound', sprintf('Input file "%s" not found starting from "%s" up to the root.', currentFileName, config.defaultLasDir));
end
fprintf('     - Resolved File Path: %s\n', resolvedFilePath);

% Extract file extension to handle .las vs .laz validation
[~, ~, fileExt] = fileparts(resolvedFilePath);

% Verify file header signature based on format (.las vs .laz)
fid = fopen(resolvedFilePath, 'r');
if fid == -1
    error('PipelineError:AccessDenied', 'Could not open point cloud file for reading.');
end
magicBytes = fread(fid, 4, 'uint8=>char')';
fclose(fid);

if strcmpi(fileExt, '.las')
    % Uncompressed LAS must start with 'LASF'
    if ~strcmp(magicBytes, 'LASF')
        error('PipelineError:InvalidHeader', 'Invalid LAS header signature ("%s"). File is corrupted.', magicBytes);
    end
    disp(' >>> [GATE PASSED] .las header signature verified.');

elseif strcmpi(fileExt, '.laz')
    % Compressed LAZ files can start with 'LASF' (standard header with LAZ compression)
    % or alternative compression signatures depending on the compressor version.
    if strcmp(magicBytes, 'LASF')
        disp(' >>> [Gate 1 PASSED] .laz header signature verified (Standard LAS header).');
    else
        % Allow alternative valid compressed signatures or check file size/readability
        fprintf('     - Notice: Non-standard LAZ signature ("%s"). Proceeding with stream validation...\n', magicBytes);
        try
            testReader = lasFileReader(resolvedFilePath);
            if testReader.Count > 0
                disp(' >>> [Gate 1 PASSED] .laz file successfully indexed by reader.');
            else
                error('LAZ file has 0 points or invalid structure.');
            end
        catch ME_laz
            error('PipelineError:InvalidLAZ', 'Invalid or corrupted .laz file: %s', ME_laz.message);
        end
    end
else
    error('PipelineError:UnsupportedFormat', 'Unsupported file extension: "%s". Expected .las or .laz.', fileExt);
end

% =================================================================
% EXTRACT WKT PROJECTION (VLR 2112) FROM SOURCE FILE
% =================================================================
wkt = '';
try
    wktReader = lasFileReader(resolvedFilePath);
    vlrRec = wktReader.readVLR(2112);
    if ~isempty(vlrRec) && ~isempty(vlrRec.Data)
        wkt = vlrRec.Data;
        fprintf('     - [CRS] WKT projection successfully extracted from source file (VLR 2112).\n');
    end
catch
    fprintf('     - [CRS NOTICE] Source file does not contain VLR 2112 projection metadata.\n');
end

% =================================================================
% PIPELINE EXECUTION (GEOREFERENCING)
% =================================================================
if config.enableGeoref
    [ptCloudProcessed, ~] = georefPipeline(resolvedFilePath, config);
    fprintf(' >>> [COMPLETED] Georeferenced point cloud generated.\n');
else
    lasReader = lasFileReader(resolvedFilePath);
    ptCloudProcessed = readPointCloud(lasReader);
    fprintf(' >>> [SKIPPED] Georeferencing skipped. Using raw .las data.\n');
end
end