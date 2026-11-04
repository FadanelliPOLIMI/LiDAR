function ptCloudMaster = step0b_reconstructTacticalBursts(config)
    % RECONSTRUCTTACTICALBURSTS Automatically scans the results folder, identifies
    % the latest tactical transmission session (within a 5-minute window),
    % filters duplicates (keeping the oldest), and merges bursts into a master point cloud.
    
    if nargin < 1 || isempty(config)
        resultsDir = fullfile('files', 'las', 'results');
    else
        resultsDir = config.defaultLasResults;
    end
    
    if ~isfolder(resultsDir)
        error('  -> [ERROR] Results directory not found: %s', resultsDir);
    end
    
    fprintf('  -> [GROUND STATION] Scanning results directory: %s\n', resultsDir);
    
    % Find all .laz or .las tactical burst files
    filePattern = fullfile(resultsDir, '*_Burst_*_Layer_*.laz');
    files = dir(filePattern);
    
    if isempty(files)
        % Try .las extension just in case
        filePattern = fullfile(resultsDir, '*_Burst_*_Layer_*.las');
        files = dir(filePattern);
    end
    
    if isempty(files)
        error('  -> [ERROR] No tactical burst files found matching pattern in %s', resultsDir);
    end
    
    % Parse timestamps and metadata from filenames
    fileRecords = []; 
    wktString = ''; % Variable to store recovered projection WKT
    
    for i = 1:length(files)
        fname = files(i).name;
        filePath = fullfile(files(i).folder, fname); % Defined FIRST so reader can use it
        
        % Extract timestamp using regular expression (14 digits: YYYYMMDDHHMMSS)
        tokens = regexp(fname, '_(\d{8})_(\d{6})\.la[sz]$', 'tokens');
        if ~isempty(tokens)
            dateStr = tokens{1}{1};
            timeStr = tokens{1}{2};
            fileTime = datenum([dateStr, timeStr], 'yyyymmddHHMMSS');
        
            % Extract Burst and Layer numbers
            burstTok = regexp(fname, '_Burst_(\d+)', 'tokens');
            layerTok = regexp(fname, '_Layer_(\d+)', 'tokens');
            burstNum = str2double(burstTok{1}{1});
            layerNum = str2double(layerTok{1}{1});
        
            % Recover WKT (VLR 2112) from available chunks if not found yet
            if isempty(wktString)
                try
                    tmpReader = lasFileReader(filePath);
                    vlrRec = tmpReader.readVLR(2112);
                    if ~isempty(vlrRec) && ~isempty(vlrRec.Data)
                        wktString = vlrRec.Data;
                        fprintf('     - [CRS] Recovered WKT projection from chunk: %s\n', fname);
                    end
                catch
                    % Ignore if chunk lacks VLR
                end
            end
            
            fileRecords(end+1).name = fname; %#ok<AGROW>
            fileRecords(end).path = filePath;        
            fileRecords(end).time = fileTime;
            fileRecords(end).burst = burstNum;
            fileRecords(end).layer = layerNum;
        end
    end
    
    if isempty(fileRecords)
        error('  -> [ERROR] Could not parse timestamps or burst IDs from filenames.');
    end
    
    % 1. Find the latest session timestamp
    allTimes = [fileRecords.time];
    latestTime = max(allTimes);
    
    % 2. Filter files belonging to the latest session (within 5 minutes / 300 seconds)
    timeWindow = 5 / (24 * 60);
    sessionIndices = find(abs(allTimes - latestTime) <= timeWindow);
    
    sessionFiles = fileRecords(sessionIndices);
    fprintf('     - Identified latest transmission session containing %d burst files.\n', length(sessionFiles));
    
    % 3. Remove duplicates (same Burst and Layer -> KEEP THE OLDEST ONE)
    [~, sortIdx] = sort([sessionFiles.time], 'ascend');
    sessionFiles = sessionFiles(sortIdx);
    
    uniqueMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    for i = 1:length(sessionFiles)
        key = sprintf('B%02d_L%02d', sessionFiles(i).burst, sessionFiles(i).layer);
        % KEEP OLDEST: Only add to map if this Burst/Layer hasn't been seen yet
        if ~isKey(uniqueMap, key)
            uniqueMap(key) = sessionFiles(i);
        end
    end
    
    uniqueKeys = keys(uniqueMap);
    fprintf('     - After duplicate filtering (keeping oldest per Burst/Layer): %d unique chunks to merge.\n', length(uniqueKeys));
    
    % 4. Ingest and Merge Point Clouds
    ptCloudMaster = pointCloud([0 0 0]); % Initialize empty
    isFirst = true;
    
    for k = 1:length(uniqueKeys)
        rec = uniqueMap(uniqueKeys{k});
        fprintf('  -> Loading & merging chunk: %s\n', rec.name);
        
        reader = lasFileReader(rec.path);
        [ptCloudChunk, ~] = readPointCloud(reader);
        
        if isFirst
            ptCloudMaster = ptCloudChunk;
            isFirst = false;
        else
            ptCloudMaster = pcmerge(ptCloudMaster, ptCloudChunk, 0.01); % 1cm voxel resolution merge
        end
    end
    
    fprintf('  -> [SUCCESS] Master point cloud reconstructed! Total Points: %d\n', ptCloudMaster.Count);
    
    % 5. Save Reconstructed Master File with WKT Injection and .prj generation
    masterFileName = sprintf('Reconstructed_Master_%s.las', datestr(now, 'yyyymmdd_HHMMSS'));
    masterFilePath = fullfile(resultsDir, masterFileName);
    
    writer = lasFileWriter(masterFilePath, 'LasVersion', '1.4');
    
    % Inject WKT VLR 2112 using helper function
    insertWKT(writer, wktString);
    
    writePointCloud(writer, ptCloudMaster);
    
    % Create complementary .prj file for GIS software (QGIS/ArcGIS)
    if ~isempty(wktString)
        prjPath = strrep(masterFilePath, '.las', '.prj');
        fid = fopen(prjPath, 'w');
        if fid ~= -1
            fprintf(fid, '%s', wktString);
            fclose(fid);
            fprintf('     - [CRS] Associated .prj file successfully created: %s\n', prjPath);
        end
    end
    
    fprintf('     - Saved master assembled point cloud to: %s\n', masterFilePath);
end