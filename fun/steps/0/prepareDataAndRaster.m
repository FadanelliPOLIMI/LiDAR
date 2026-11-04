function [ctx, config] = prepareDataAndRaster(lasFilePath, config, varargin)
    % PREPAREDATAANDRASTER Loads LAS, segments terrain, and creates rasters.
    % JUST TO START WITH THE RIGHT REFERENCE FILES

    % 1. Ensure config exists
    if nargin < 2 || isempty(config)
        config = struct();
    end

    % 2. Set default fallbacks if fields don't exist in config yet
    if ~isfield(config, 'BaseRes'), config.BaseRes = 1200; end
    if ~isfield(config, 'BufferLimit'), config.BufferLimit = 0.10; end
    if ~isfield(config, 'MinHeightLimit'), config.MinHeightLimit = 0.10; end
    if ~isfield(config, 'MaxDepthLimit'), config.MaxDepthLimit = -0.10; end
    if ~isfield(config, 'MaxRatioHOG'), config.MaxRatioHOG = 0.90; end
    if ~isfield(config, 'MaxDistRANSAC'), config.MaxDistRANSAC = 5.0; end
    if ~isfield(config, 'alignmentMode'), config.alignmentMode = 'manual'; end

    % 3. Set up input parser using current config values as defaults
    p = inputParser;
    addParameter(p, 'BaseRes', config.BaseRes, @isnumeric);
    addParameter(p, 'BufferLimit', config.BufferLimit, @isnumeric);
    addParameter(p, 'MinHeightLimit', config.MinHeightLimit, @isnumeric);
    addParameter(p, 'MaxDepthLimit', config.MaxDepthLimit, @isnumeric);
    addParameter(p, 'MaxRatioHOG', config.MaxRatioHOG, @isnumeric);
    addParameter(p, 'MaxDistRANSAC', config.MaxDistRANSAC, @isnumeric);
    addParameter(p, 'alignmentMode', config.alignmentMode, @ischar);

    parse(p, varargin{:});

    % 4. Dynamically update config fields with parsed results (preserves other fields like 'mode')
    parsedFields = fieldnames(p.Results);
    for i = 1:length(parsedFields)
        config.(parsedFields{i}) = p.Results.(parsedFields{i});
    end
    
    % Extract directory, base name, and extension from the resolved path
    [inputDir, fileBaseName, ~] = fileparts(lasFilePath);
    if isempty(inputDir)
        inputDir = pwd;
    end

    disp(['>>> [STEP] Loading LAS file: ', lasFilePath]);
    lasReader = lasFileReader(lasFilePath);
    ptCloud = readPointCloud(lasReader);

    % Terrain segmentation (SMRF) and obstacle classification
    disp('>>> [STEP] Performing terrain segmentation (SMRF) and obstacle classification...');
    [groundIdx, nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(ptCloud);
    clusterLabels = ones(ptCloud.Count, 1);
    labels = helperSegment(nonGroundPtCloud); % Classification Algo
    clusterLabels(~groundIdx) = labels;
    vegetationIdx = clusterLabels == 2;
    buildingIdx = clusterLabels == 3;
    
    cleanGroundPtCloud = removeInvalidPoints(groundPtCloud);
    groundPoints = cleanGroundPtCloud.Location;
    [uniqueZ, uniqueXY] = groupsummary(groundPoints(:,3), groundPoints(:,1:2), @mean); 
    uniqueXY = [uniqueXY{:}];
    F = scatteredInterpolant(double(uniqueXY), double(uniqueZ), "natural");
    z_all = ptCloud.Location(:,3);
    z_ground_interp = F(ptCloud.Location(:,1), ptCloud.Location(:,2));
    residualZ = z_all - z_ground_interp;
    anomalyIdx = (residualZ > config.MinHeightLimit) | (residualZ < config.MaxDepthLimit) | vegetationIdx | buildingIdx;

    % Robust CRS Reading with Fallback
    try
        projCRSData = readCRS(lasReader);
    catch
        warning('>>> [CRS WARNING] Could not read CRS directly from lasReader. Attempting fallback lookup...');
        projCRSData = []; % Or a default projected CRS if needed
    end
    [lat_orig, lon_orig] = projinv(projCRSData, ptCloud.Location(:,1), ptCloud.Location(:,2));

    % BBOX & Geographic Aspect Ratio
    validPts = ~isnan(lat_orig) & ~isnan(lon_orig);
    latMin_raw = min(lat_orig(validPts)); latMax_raw = max(lat_orig(validPts));
    lonMin_raw = min(lon_orig(validPts)); lonMax_raw = max(lon_orig(validPts));
    latBuffer = (latMax_raw - latMin_raw) * config.BufferLimit; 
    lonBuffer = (lonMax_raw - lonMin_raw) * config.BufferLimit;
    if latBuffer == 0, latBuffer = 0.001; end
    if lonBuffer == 0, lonBuffer = 0.001; end
    latMin = latMin_raw - latBuffer; latMax = latMax_raw + latBuffer;
    lonMin = lonMin_raw - lonBuffer; lonMax = lonMax_raw + lonBuffer;
    
    meanLat = mean([latMin, latMax]);
    % Mathematical formula from lat to meters
    metersPerLat = 111132.92 - 559.82 * cos(2*deg2rad(meanLat)) + 1.175 * cos(4*deg2rad(meanLat));
    metersPerLon = 111412.84 * cos(deg2rad(meanLat)) - 93.5 * cos(3*deg2rad(meanLat));
    spanY_meters = (latMax - latMin) * metersPerLat;
    spanX_meters = (lonMax - lonMin) * metersPerLon;
    
    if spanY_meters > spanX_meters
        imgHeight = config.BaseRes;
        imgWidth = round(config.BaseRes * (spanX_meters / spanY_meters));
    else
        imgWidth = config.BaseRes;
        imgHeight = round(config.BaseRes * (spanY_meters / spanX_meters));
    end
    imgSize = [imgHeight, imgWidth]; 

    % Multi-modal Rasters Generation
    disp('>>> [STEP] Generating multi-modal 2D raster images...');
    intensityImg = zeros(imgSize, 'uint8');
    dsmImg = zeros(imgSize, 'uint8');
    anomImg = zeros(imgSize, 'uint8');
    
    rNormLat_all = (lat_orig - latMin) / (latMax - latMin);
    rNormLon_all = (lon_orig - lonMin) / (lonMax - lonMin);
    px_all = round(rNormLon_all * (imgWidth - 1)) + 1;
    py_all = round((1 - rNormLat_all) * (imgHeight - 1)) + 1; 
    inBounds_all = validPts & px_all >= 1 & px_all <= imgWidth & py_all >= 1 & py_all <= imgHeight;
    
    if isprop(ptCloud, 'Intensity') && ~isempty(ptCloud.Intensity)
        int_vals = double(ptCloud.Intensity);
        if any(inBounds_all)
            linearIdx = sub2ind(imgSize, py_all(inBounds_all), px_all(inBounds_all));
            intAcc = accumarray(linearIdx, int_vals(inBounds_all), [prod(imgSize), 1], @max, 0);
            intensityRaw = uint8(rescale(reshape(intAcc, imgSize), 0, 255));
            intensityImg = imadjust(intensityRaw, stretchlim(intensityRaw), []);
        end
    end
    
    validGeo = inBounds_all & (vegetationIdx | buildingIdx);
    if any(validGeo)
        z_c = ptCloud.Location(validGeo, 3);
        linearIdx = sub2ind(imgSize, py_all(validGeo), px_all(validGeo));
        dsmAcc = accumarray(linearIdx, z_c, [prod(imgSize), 1], @max, 0);
        dsmImg = uint8(rescale(reshape(dsmAcc, imgSize), 0, 255));
    end
    
    validAnom = inBounds_all & anomalyIdx;
    if any(validAnom)
        z_a = ptCloud.Location(validAnom, 3);
        linearIdx = sub2ind(imgSize, py_all(validAnom), px_all(validAnom));
        anomAcc = accumarray(linearIdx, z_a, [prod(imgSize), 1], @max, 0);
        anomImg = uint8(rescale(reshape(anomAcc, imgSize), 0, 255));
    end
    
    lidarCompositeRGB = cat(3, intensityImg, dsmImg, anomImg);

    % Fetch Reference Satellite Basemap
    disp('>>> Acquiring reference satellite image (ArcGIS World Imagery)...');
    figSat = figure('Visible', 'off', 'Position', [100, 100, imgWidth, imgHeight]);
    geax = geoaxes('Parent', figSat, 'Units', 'normalized', 'Position', [0 0 1 1]);
    geolimits(geax, [latMin latMax], [lonMin lonMax]);
    geobasemap(geax, 'satellite');
    geax.Visible = 'on'; 
    pause(2);
    drawnow;
    frame = getframe(figSat);
    try close(figSat); catch; end
    
    satImg = imresize(frame.cdata, imgSize, 'bilinear');
    if ndims(satImg) == 2
        satImg = cat(3, satImg, satImg, satImg);
    elseif size(satImg, 3) > 3
        satImg = satImg(:,:,1:3);
    end
    satImg = im2uint8(satImg);
    
    % Pack everything into context struct
    ctx.lasName = lasFilePath;
    ctx.fileBaseName = fileBaseName;
    ctx.inputDir = inputDir;
    ctx.resultsDir = config.defaultLasResults;
    ctx.lasReader = lasReader;
    ctx.ptCloud = ptCloud;
    ctx.projCRSData = projCRSData;
    ctx.lat_orig = lat_orig;
    ctx.lon_orig = lon_orig;
    ctx.latMin = latMin; ctx.latMax = latMax;
    ctx.lonMin = lonMin; ctx.lonMax = lonMax;
    ctx.spanX_meters = spanX_meters;
    ctx.spanY_meters = spanY_meters;
    ctx.metersPerLat = metersPerLat;
    ctx.metersPerLon = metersPerLon;
    ctx.imgWidth = imgWidth;
    ctx.imgHeight = imgHeight;
    ctx.lidarCompositeRGB = lidarCompositeRGB;
    ctx.satImg = satImg;
    ctx.satGray = rgb2gray(satImg);
    ctx.lidarGray = rgb2gray(lidarCompositeRGB);
    ctx.anomalyIdx = anomalyIdx;
end

function labels = helperSegment(nonGroundPtCloud)
    neighbors = 10;
    [normals, curvatures, neighInds] = helperExtractFeatures(nonGroundPtCloud, neighbors);
    normalThresh = 0.75;
    curveThresh = 0.035;
    labels = helperClassify(normals, curvatures, neighInds, normalThresh, curveThresh);
    labels = labels + 1;
end

function labels = helperClassify(normals, curvatures, neighInds, norThres, curThres)
    [~, sortedIds] = sort(curvatures);
    minClusterSize = 100;
    numPts = size(normals, 1);
    numberOfNeighbors = size(neighInds, 1);
    notVisited = true(numPts, 1);
    seedNotAdded = true(numPts, 1);
    labels = ones(numPts, 1);
    labels(isnan(curvatures)) = 0;
    numValidPts = sum(~isnan(curvatures));
    for i = 1 : numValidPts
        if notVisited(sortedIds(i))
            notVisited(sortedIds(i)) = false;
            seedPtIdx = sortedIds(i);
            regionPts = [];
            while ~isempty(seedPtIdx)
                curIdx = seedPtIdx(1);
                for j = 1:numberOfNeighbors
                    idx = neighInds(j, curIdx);
                    if notVisited(idx)
                        normDiff = abs(dot(normals(curIdx, :), normals(idx, :)));
                        if normDiff >= norThres
                            notVisited(idx) = false;
                            regionPts(end+1) = idx;
                        end
                        if curvatures(idx) <= curThres && seedNotAdded(idx)
                            seedNotAdded(idx) = false;
                            seedPtIdx(end+1) = idx;
                        end
                    end
                end
                seedPtIdx(1) = [];
            end
            if length(regionPts) > minClusterSize
                labels(regionPts) = 2;
            end
        end
    end
end