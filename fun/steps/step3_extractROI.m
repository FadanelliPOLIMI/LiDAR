function [ptCloudExtracted, ptCloudAligned, config, stage3Metrics] = step3_extractROI(ptCloudFiltered, config)
    % STEP3_EXTRACTROI - Stage 3 Road Extraction & Geometric Alignment
    % Optimized for Edge (Jetson Nano) by bypassing redundant masks in 'auto' mode.
    
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    
    % Set defaults if missing from config
    if ~isfield(config, 'extractionMode'), config.extractionMode = 'interactive'; end
    if ~isfield(config, 'roiType'), config.roiType = 'rectangle'; end
    
    ticTotal = tic;
    
    fprintf('\n--------------------------------------------\n');
    fprintf('STAGE 3: ROAD EXTRACTION & GEOMETRIC ALIGNMENT\n');
    fprintf('--------------------------------------------\n');
    
    % Step 3A: Align road via 2D PCA
    try
        [ptCloudAligned, geoTransform, xMin_auto, yMin_auto, xMax_auto, yMax_auto, pcaFailed] = ...
            alignRoadPCA(ptCloudFiltered, config);
        
        config.geoTransform = geoTransform;
        fprintf('  -> [GEO-TRACKING] geoTransform successfully captured and stored in config.\n');
        
    catch ME
        warning('Step3A alignment failed (%s). Falling back to unaligned filtered cloud.', ME.message);
        % Extract global locations
        locs = ptCloudFiltered.Location;
        xMin_auto = min(locs(:,1)); xMax_auto = max(locs(:,1));
        yMin_auto = min(locs(:,2)); yMax_auto = max(locs(:,2));

        % Translate to local origin (0,0) AND cast to single precision
        coords_local_single = single([locs(:,1) - xMin_auto, locs(:,2) - yMin_auto, locs(:,3)]);

        if isprop(ptCloudFiltered, 'Color') && ~isempty(ptCloudFiltered.Color)
            ptCloudAligned = pointCloud(coords_local_single, 'Color', ptCloudFiltered.Color);
        else
            ptCloudAligned = pointCloud(coords_local_single);
        end

        % Maintain single precision on normals if present
        if isprop(ptCloudFiltered, 'Normal') && ~isempty(ptCloudFiltered.Normal)
            ptCloudAligned.Normal = single(ptCloudFiltered.Normal);
        end

        geoTransform = struct('centroid', [xMin_auto, yMin_auto], 'theta', 0, 'R', eye(2), 'xOffset', 0, 'yOffset', 0);
        config.geoTransform = geoTransform;
        pcaFailed = true;
    end
    
    % --- HARDWARE & GEOMETRY ADAPTIVE OVERRIDES ---
    if strcmpi(config.mode, 'edge')
        fprintf('  -> [EDGE MODE] Headless environment detected. Forcing auto-bounding box (full file limits).\n');
        config.extractionMode = 'auto';
        config.roiType = 'rectangle';
    else
        if pcaFailed
            fprintf('  -> [LAPTOP AUTO-CORRECTION] PCA failed (complex geometry). Switching to INTERACTIVE POLYGON mode.\n');
            config.extractionMode = 'interactive';
            config.roiType = 'polygon';
        end
    end
    
    % Step 3B: Handle extraction mode logic & Selective Masking
    switch lower(config.extractionMode)
        case 'auto'
            fprintf('  -> [MODE: AUTO] Using full spatial limits. Bypassing spatial filtering mask for Edge efficiency.\n');
            config.xmin = xMin_auto; config.xmax = xMax_auto;
            config.ymin = yMin_auto; config.ymax = yMax_auto;
            
            % --- EDGE OPTIMIZATION: Direct pass-through (Zero memory allocation) ---
            ptCloudExtracted = ptCloudAligned;
            
        case 'interactive'
            fprintf('  -> [MODE: INTERACTIVE] Opening interactive selection tool (Type: %s)...\n', upper(config.roiType));
            config = selectSpatialLimitsInteractive(ptCloudAligned, config);
            
            % Apply mask only when user interactively cropped
            if strcmpi(config.roiType, 'polygon') && isfield(config, 'roiPolygon') && ~isempty(config.roiPolygon)
                inMask = inpolygon(ptCloudAligned.Location(:,1), ptCloudAligned.Location(:,2), ...
                    config.roiPolygon(:,1), config.roiPolygon(:,2));
                ptCloudExtracted = select(ptCloudAligned, inMask);
            else
                loc = ptCloudAligned.Location;
                inMask = (loc(:,1) >= config.xmin & loc(:,1) <= config.xmax & ...
                          loc(:,2) >= config.ymin & loc(:,2) <= config.ymax);
                ptCloudExtracted = select(ptCloudAligned, inMask);
            end
            
        case 'preset'
            fprintf('  -> [MODE: PRESET] Using pre-existing config spatial limits.\n');
            if ~all(isfield(config, {'xmin', 'xmax', 'ymin', 'ymax'}))
                error('PipelineError:MissingPreset', 'extractionMode is "preset" but xmin/xmax/ymin/ymax are not defined in config.');
            end
            loc = ptCloudAligned.Location;
            inMask = (loc(:,1) >= config.xmin & loc(:,1) <= config.xmax & ...
                      loc(:,2) >= config.ymin & loc(:,2) <= config.ymax);
            ptCloudExtracted = select(ptCloudAligned, inMask);
            
        otherwise
            error('PipelineError:InvalidMode', 'Unknown config.extractionMode: %s', config.extractionMode);
    end
    
    totalElapsedTime = toc(ticTotal);
    
    % --- STAGE 3 VISUALIZATION (Optional) ---
    if isfield(config, 'visible') && config.visible
        fprintf('  -> [VISUALIZATION] Rendering Stage 3 extracted point cloud...\n');
        figure('Name', 'Stage 3: Extracted Road Region of Interest', 'Color', 'w');
        pcshow(ptCloudExtracted);
        title(sprintf('Stage 3 Extracted ROI (%d points)', ptCloudExtracted.Count));
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        drawnow;
    end
    
    % Populate metrics output
    stage3Metrics = struct('initialCount', ptCloudFiltered.Count, ...
        'finalCount', ptCloudExtracted.Count, ...
        'retentionRate', (ptCloudExtracted.Count / max(1, ptCloudFiltered.Count)) * 100, ...
        'totalExecutionTime', totalElapsedTime, ...
        'pcaFailed', pcaFailed, ...
        'geoTransform', config.geoTransform);
    
    fprintf('  -> Final Extracted Points: %d\n', ptCloudExtracted.Count);
    fprintf('  -> Stage 3 Execution Time: %.2f seconds\n', totalElapsedTime);
    fprintf('--------------------------------------------\n');
end