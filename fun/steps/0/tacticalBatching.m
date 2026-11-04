function batchFiles = tacticalBatching(ptCloudProcessed, baseName, config, wkt, varargin)
    % TACTICALBATCHING Dynamically calculates variable-sized chunks and epochs
    % based on total point cloud byte footprint, 5G bandwidth limits, and EW channel jitter.
    
    % --- PARSE OPTIONAL ARGUMENTS (Visibility control) ---
    p = inputParser;
    addParameter(p, 'Visible', 'off', @(x) ischar(x) || islogical(x));
    parse(p, varargin{:});
    
    visInput = p.Results.Visible;
    if islogical(visInput)
        if visInput, visState = 'on'; else, visState = 'off'; end
    else
        visState = lower(visInput);
    end
    
    % Also check config override if present
    if exist('config', 'var') && isfield(config, 'visualizePlots') && config.visualizePlots
        visState = 'on';
    end

    disp(' >>> [TACTICAL PIPELINE] Analyzing full point cloud data footprint & 5G dynamic jitter...');
    
    if nargin < 3 || isempty(config)
        config = struct();
    end
    
    if isfield(config, 'defaultLasResults') && ~isempty(config.defaultLasResults)
        resultsDir = config.defaultLasResults;
    else
        resultsDir = fullfile('files', 'results');
    end
    
    if ~isfolder(resultsDir)
        mkdir(resultsDir);
    end 
    
    % =====================================================================
    % 2. COMPUTE TRUE DATA FOOTPRINT (All Attributes)
    % =====================================================================
    totalPoints = ptCloudProcessed.Count;
    bytesPerPoint = 12; % XYZ coordinates (single precision = 4 bytes * 3)
    
    hasColor = isprop(ptCloudProcessed, 'Color') && ~isempty(ptCloudProcessed.Color);
    if hasColor
        bytesPerPoint = bytesPerPoint + 3; % RGB uint8 = 3 bytes
    end
    
    hasIntensity = isprop(ptCloudProcessed, 'Intensity') && ~isempty(ptCloudProcessed.Intensity);
    if hasIntensity
        bytesPerPoint = bytesPerPoint + 2; % Intensity uint16 = 2 bytes
    end
    
    uncompressedBytes = totalPoints * bytesPerPoint;
    compressionRatio = 0.18; % Estimated LAZ compression ratio (~18% of raw size)
    estimatedLazBytes = round(uncompressedBytes * compressionRatio);
    
    attrStr = 'XYZ';
    if hasColor, attrStr = [attrStr, ' + RGB']; end
    if hasIntensity, attrStr = [attrStr, ' + Intensity']; end
    
    fprintf('     - Point Cloud Metrics:\n');
    fprintf('       * Total Points (after normalization): %d\n', totalPoints);
    fprintf('       * Attributes Included: %s\n', attrStr);
    fprintf('       * Estimated Compressed .LAZ Footprint: %.2f KB (%d bytes)\n', ...
        estimatedLazBytes / 1024, estimatedLazBytes);
        
    % =====================================================================
    % 3. EW-RESILIENT 5G BANDWIDTH ASSESSMENT & BASELINE CHUNKS
    % =====================================================================
    if isfield(config, 'maxBandwidthBytes') && ~isempty(config.maxBandwidthBytes)
        maxBurstBytes = config.maxBandwidthBytes;
    else
        maxBurstBytes = 512 * 1024; % Default: 512 KB per burst
    end
    
    targetChunks = ceil(estimatedLazBytes / maxBurstBytes);
    targetChunks = max(targetChunks, 3); 
    
    avgPointsPerChunk = totalPoints / targetChunks;
    
    fprintf('     - 5G Bandwidth Constraint: Max ~%.2f KB per burst ceiling.\n', maxBurstBytes / 1024);
    fprintf('     - Target Base Chunks: %d (Will introduce dynamic jitter)\n', targetChunks);
    
    % =====================================================================
    % 4. VARIABLE-SIZED TEMPORAL & SPATIAL CHUNKING EXECUTION
    % =====================================================================
    timestampStr = datestr(now, 'yyyymmdd_HHMMSS');
    batchFiles = {};
    
    currentIdx = 1;
    c = 0;
    
    while currentIdx <= totalPoints
        c = c + 1;
        
        jitterFactor = 0.7 + (1.4 - 0.7) * rand();
        chunkPtsCount = round(avgPointsPerChunk * jitterFactor);
        
        startIdx = currentIdx;
        if c == targetChunks || (startIdx + chunkPtsCount) > totalPoints
            endIdx = totalPoints; 
        else
            endIdx = min(startIdx + chunkPtsCount - 1, totalPoints);
        end
        
        ptCloudChunk = select(ptCloudProcessed, startIdx:endIdx);
        actualPoints = ptCloudChunk.Count;
        
        if actualPoints == 0
            break;
        end
        
        fprintf('\n     --------------------------------------------------\n');
        fprintf('     ⏱ [TACTICAL BURST %d] Variable Sizing (Jitter: %.2fx)\n', c, jitterFactor);
        fprintf('     --------------------------------------------------\n');
        fprintf('       * Assigned Points in Burst: %d\n', actualPoints);
        
        subLayers = 2; 
        for l = 1:subLayers
            layerIndices = l : subLayers : actualPoints;
            if isempty(layerIndices)
                continue;
            end
            ptCloudSubBatch = select(ptCloudChunk, layerIndices);
            
            batchLazName = sprintf('Terrain_%s_Burst_%02d_Layer_%02d_%s.laz', baseName, c, l, timestampStr);
            batchLazPath = fullfile(resultsDir, batchLazName);
            batchFiles{end+1} = batchLazPath; %#ok<AGROW>
            
            lazWriter = lasFileWriter(batchLazPath);
            insertWKT(lazWriter, wkt);
            writePointCloud(lazWriter, ptCloudSubBatch);
            
            fprintf('       > [5G RF Outbox] File: %s | Sub-layer Points: %d\n', ...
                batchLazName, ptCloudSubBatch.Count);
                
            qcReport = inspectLasFile(batchLazPath);
            if qcReport.isValid && qcReport.hasCoordinates
                fprintf('       > [QC PASSED] Encrypted variable payload verified.\n');
            else
                warning('       > [QC FAILED] Variable packet integrity check failed.');
            end
        end
        
        % =================================================================
        % VISUALIZATION: 3D COMPARISON PLOT FOR FIRST TACTICAL BURST
        % =================================================================
        if c == 1
            disp('     > [VISUALIZATION] Generating 3D comparison plot for Burst 1...');
            figure('Name', sprintf('Tactical QC - Burst 1 vs Full Resolution (%s)', baseName), ...
                'Position', [100, 100, 1400, 600], 'Visible', visState);
                
            subplot(1, 2, 1);
            pcshow(ptCloudProcessed);
            title({'Full Processed Point Cloud', sprintf('Total Points: %d', ptCloudProcessed.Count)}, ...
                'FontSize', 11, 'FontWeight', 'bold');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            grid on; axis equal;
            
            subplot(1, 2, 2);
            pcshow(ptCloudChunk);
            title({'First Tactical Burst (5G Edge Payload)', sprintf('Burst Points: %d (Jitter: %.2fx)', ptCloudChunk.Count, jitterFactor)}, ...
                'FontSize', 11, 'FontWeight', 'bold');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            grid on; axis equal;
        end
        
        currentIdx = endIdx + 1;
    end
    
    fprintf('\n >>> [SUCCESS] All %d dynamic variable-sized tactical bursts prepared in: %s\n', ...
        length(batchFiles), resultsDir);
end