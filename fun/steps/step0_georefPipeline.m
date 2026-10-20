function [ptCloudColored, outputLasFile] = step0_georefPipeline(lasName, varargin)
    % GEOREFPIPELINE Pipeline automatica e interattiva LiDAR to Adams (Caserta)
    % 
    % Utilizzo:
    %   [ptCloudColored, outFile] = georefPipeline('Alta Frequenza 5 cm.las');
    %

    % Gestione dei parametri di configurazione di default (facilmente personalizzabili)
    p = inputParser;
    addParameter(p, 'BaseRes', 900, @isnumeric);
    addParameter(p, 'BufferLimit', 0.10, @isnumeric);
    addParameter(p, 'MinHeightLimit', 0.10, @isnumeric);
    addParameter(p, 'MaxDepthLimit', -0.10, @isnumeric);
    addParameter(p, 'MaxRatioHOG', 0.90, @isnumeric);
    addParameter(p, 'MaxDistRANSAC', 5.0, @isnumeric);
    parse(p, varargin{:});
    config = p.Results;

    % Dichiarazione variabili globali per la GUI interattiva di fallback
    global totalShiftEast_m totalShiftNorth_m spanX_meters spanY_meters imgWidth imgHeight lidarCompositeRGB satImg figRef stepPopup sessionActive;

    %% =========================================================================
    % 1. CARICAMENTO DINAMICO DEL FILE LAS E METADATI WKT
    % =========================================================================
    currentDir = pwd; 
    % [parentDir, ~] = fileparts(currentDir);
    % [baseDir, ~] = fileparts(parentDir);

    lasfile = fullfile(currentDir, lasName);
    [~, fileBaseName, ~] = fileparts(lasName);

    disp(['>>> [1/5] Caricamento file LAS: ', lasName]);
    lasReader = lasFileReader(lasfile);
    ptCloud = readPointCloud(lasReader);

    % Segmentazione terreno (SMRF) e classificazione ostacoli
    disp('>>> [2/5] Segmentazione terreno (SMRF) e classificazione ostacoli...');
    [groundIdx, nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(ptCloud);
    clusterLabels = ones(ptCloud.Count,1);
    labels = helperSegment(nonGroundPtCloud);
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

    projCRSData = readCRS(lasReader);
    [lat_orig, lon_orig] = projinv(projCRSData, ptCloud.Location(:,1), ptCloud.Location(:,2));

    %% =========================================================================
    % CALCOLO BBOX E ASPECT RATIO GEOGRAFICO REALE
    % =========================================================================
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

    metersPerLat = 111132.92 - 559.82 * cos(2*deg2rad(meanLat)) + 1.175 * cos(4*deg2rad(meanLat));
    metersPerLon = 111412.84 * cos(deg2rad(meanLat)) - 93.5 * cos(3*deg2rad(meanLat));

    spanY_meters = (latMax - latMin) * metersPerLat;
    spanX_meters = (lonMax - lonMin) * metersPerLon;

    if spanY_meters > spanX_meters
        imgHeight = config.BaseRes;
        imgWidth = max(200, round(config.BaseRes * (spanX_meters / spanY_meters)));
    else
        imgWidth = config.BaseRes;
        imgHeight = max(200, round(config.BaseRes * (spanY_meters / spanX_meters)));
    end
    imgSize = [imgHeight, imgWidth]; 

    %% =========================================================================
    % GENERAZIONE IMMAGINI RASTER MULTI-MODALI
    % =========================================================================
    disp('>>> [3/5] Generazione raster 2D multi-modali...');
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

    % Acquisizione mappa satellitare di riferimento
    disp('>>> Acquisizione immagine satellitare (ArcGIS World Imagery)...');
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
    satGray = rgb2gray(satImg);
    lidarGray = rgb2gray(lidarCompositeRGB);

    alignmentAccepted = false;
    totalShiftEast_m = 0; 
    totalShiftNorth_m = 0; 

    %% =========================================================================
    % [4/5] PIPELINE DI ALLINEAMENTO IN SERIE (DL -> HOG -> imregcorr)
    % =========================================================================
    disp('>>> [4/5] Avvio sequenza di georeferenziazione automatica...');

    % --- STEP A: DEEP LEARNING (ResNet-50) ---
    if ~alignmentAccepted
        disp(' -> Tentativo 1/3: Deep Learning (ResNet-50)...');
        try
            net = resnet50;
            imgRGB_LiDAR = cat(3, intensityImg, intensityImg, intensityImg);
            imgRGB_Sat   = cat(3, satGray, satGray, satGray);
            featuresLiDAR_CNN = activations(net, imgRGB_LiDAR, 'activation_40_relu', 'OutputAs', 'channels');
            featuresSat_CNN   = activations(net, imgRGB_Sat,   'activation_40_relu', 'OutputAs', 'channels');
            featMapLiDAR = mean(featuresLiDAR_CNN, 3);
            featMapSat   = mean(featuresSat_CNN, 3);
            
            tformDeep = imregcorr(featMapLiDAR, featMapSat, 'translation');
            shiftE_deep = (tformDeep.T(3, 1) / size(featMapSat, 2)) * spanX_meters;
            shiftN_deep = -(tformDeep.T(3, 2) / size(featMapSat, 1)) * spanY_meters;
            
            figPrev = figure('Name', 'Verifica - Deep Learning', 'Position', [0, 0, 800, 700]);
            movegui(figPrev, 'center'); 
            pxPrev = round((shiftE_deep / spanX_meters) * imgWidth);
            pyPrev = round(-(shiftN_deep / spanY_meters) * imgHeight);
            prevLiDAR = imtranslate(lidarCompositeRGB, [pxPrev, pyPrev], 'FillValues', 0);
            imshowpair(satImg, prevLiDAR, 'blend');
            title(sprintf('Verifica Deep Learning\nEst: %.2f m | Nord: %.2f m', shiftE_deep, shiftN_deep));
            drawnow;
            
            sceltaDeep = questdlg('Il posizionamento Deep Learning è corretto?', 'Conferma DL', 'Sì', 'No', 'Sì');
            try close(figPrev); catch; end
            
            if strcmp(sceltaDeep, 'Sì')
                totalShiftEast_m = shiftE_deep;
                totalShiftNorth_m = shiftN_deep;
                alignmentAccepted = true;
                disp(' >>> [OK] Deep Learning accettato.');
            end
        catch ME
            disp([' >>> [Avviso DL]: ', ME.message]);
        end
    end

    % --- STEP B: FEATURE MATCHING (HOG Adattivo) ---
    if ~alignmentAccepted
        disp(' -> Tentativo 2/3: Feature Matching (HOG Adattivo)...');
        try
            ptsLiDAR = detectMinEigenFeatures(intensityImg, 'ROI', [5, 5, size(intensityImg,2)-10, size(intensityImg,1)-10], 'MinQuality', 0.005);
            ptsSat   = detectMinEigenFeatures(satGray,    'ROI', [5, 5, size(satGray,2)-10,    size(satGray,1)-10],    'MinQuality', 0.005);
            
            edgesLiDAR = edge(intensityImg, 'Canny', [0.05 0.15]);
            edgesSat   = edge(satGray, 'Canny', [0.05 0.15]);
            [yL, xL] = find(edgesLiDAR); [yS, xS] = find(edgesSat);
            
            stepL = max(1, floor(length(xL)/500));
            stepS = max(1, floor(length(xS)/500));
            ptsLiDAR_edges = cornerPoints([xL(1:stepL:end), yL(1:stepL:end)]);
            ptsSat_edges   = cornerPoints([xS(1:stepS:end), yS(1:stepS:end)]);
            
            allPtsLiDAR = [ptsLiDAR; ptsLiDAR_edges];
            allPtsSat   = [ptsSat; ptsSat_edges];
            
            [featuresLiDAR, validPtsLiDAR] = extractHOGFeatures(intensityImg, allPtsLiDAR, 'CellSize', [12 12], 'BlockSize', [2 2], 'NumBins', 9);
            [featuresSat, validPtsSat]     = extractHOGFeatures(satGray, allPtsSat, 'CellSize', [12 12], 'BlockSize', [2 2], 'NumBins', 9);
            
            indexPairs = matchFeatures(featuresLiDAR, featuresSat, 'Unique', false, 'MaxRatio', config.MaxRatioHOG);
            
            if ~isempty(indexPairs) && size(indexPairs, 1) >= 4
                matchedPointsLiDAR = validPtsLiDAR(indexPairs(:,1), :);
                matchedPointsSat   = validPtsSat(indexPairs(:,2), :);
                
                [tformFeature, inliers] = estimateGeometricTransform2D(matchedPointsLiDAR, matchedPointsSat, 'translation', ...
                    'Confidence', 99.0, 'MaxDistance', config.MaxDistRANSAC);
                
                shiftE_feat = (tformFeature.T(3, 1) / imgWidth) * spanX_meters;
                shiftN_feat = -(tformFeature.T(3, 2) / imgHeight) * spanY_meters;
                
                figPrev = figure('Name', 'Verifica - HOG Adattivo', 'Position', [0, 0, 800, 700]);
                movegui(figPrev, 'center'); 
                pxPrev = round((shiftE_feat / spanX_meters) * imgWidth);
                pyPrev = round(-(shiftN_feat / spanY_meters) * imgHeight);
                prevLiDAR = imtranslate(lidarCompositeRGB, [pxPrev, pyPrev], 'FillValues', 0);
                imshowpair(satImg, prevLiDAR, 'blend');
                title(sprintf('Verifica HOG (Inliers: %d)\nEst: %.2f m | Nord: %.2f m', sum(inliers), shiftE_feat, shiftN_feat));
                drawnow;
                
                sceltaFeat = questdlg('Il posizionamento HOG è corretto?', 'Conferma HOG', 'Sì', 'No', 'Sì');
                try close(figPrev); catch; end
                    
                if strcmp(sceltaFeat, 'Sì')
                    totalShiftEast_m = shiftE_feat;
                    totalShiftNorth_m = shiftN_feat;
                    alignmentAccepted = true;
                    disp(' >>> [OK] HOG accettato.');
                end
            end
        catch ME
            disp([' >>> [Avviso HOG]: ', ME.message]);
        end
    end

    % --- STEP C: IMREGCORR ---
    if ~alignmentAccepted
        disp(' -> Tentativo 3/3: Correlazione raster diretta (imregcorr)...');
        try
            tformReg = imregcorr(lidarGray, satGray, 'translation');
            shiftE_reg = (tformReg.T(3, 1) / imgWidth) * spanX_meters;
            shiftN_reg = -(tformReg.T(3, 2) / imgHeight) * spanY_meters; 
            
            figPrev = figure('Name', 'Verifica - imregcorr', 'Position', [0, 0, 800, 700]);
            movegui(figPrev, 'center'); 
            pxPrev = round((shiftE_reg / spanX_meters) * imgWidth);
            pyPrev = round(-(shiftN_reg / spanY_meters) * imgHeight);
            prevLiDAR = imtranslate(lidarCompositeRGB, [pxPrev, pyPrev], 'FillValues', 0);
            imshowpair(satImg, prevLiDAR, 'blend');
            title(sprintf('Verifica imregcorr\nEst: %.2f m | Nord: %.2f m', shiftE_reg, shiftN_reg));
            drawnow;
            
            sceltaReg = questdlg('Il posizionamento imregcorr è corretto?', 'Conferma imregcorr', 'Sì', 'No', 'Sì');
            try close(figPrev); catch; end
                
            if strcmp(sceltaReg, 'Sì')
                totalShiftEast_m = shiftE_reg;
                totalShiftNorth_m = shiftN_reg;
                alignmentAccepted = true;
                disp(' >>> [OK] imregcorr accettato.');
            end
        catch ME
            disp([' >>> [Avviso imregcorr]: ', ME.message]);
        end
    end

    % --- STEP D: FALLBACK MANUALE INTERATTIVO ---
    if ~alignmentAccepted
        disp(' >>> Tutti i metodi automatici rifiutati. Apertura pannello di controllo manuale...');
        figRef = figure('Name', 'Controllo Interattivo Sovrapposizione LiDAR su Satellite', 'Position', [50, 50, 900, 800]);
        figCtrl = figure('Name', 'Pannello di Spostamento', 'Position', [980, 400, 300, 350], 'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');
        uicontrol('Style', 'text', 'Position', [20, 300, 260, 30], 'String', 'Scegli Step di Spostamento:', 'FontSize', 10, 'FontWeight', 'bold');
        stepPopup = uicontrol('Style', 'popupmenu', 'Position', [20, 270, 260, 30], 'String', {'0.1 Metri', '0.5 Metri', '1 Metro', '5 Metri', '10 Metri'}, 'Value', 3);
        uicontrol('Style', 'pushbutton', 'String', '▲ NORD', 'Position', [110, 200, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(0, getStepVal()));
        uicontrol('Style', 'pushbutton', 'String', '▼ SUD', 'Position', [110, 110, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(0, -getStepVal()));
        uicontrol('Style', 'pushbutton', 'String', '◄ OVEST', 'Position', [20, 155, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(-getStepVal(), 0));
        uicontrol('Style', 'pushbutton', 'String', '► EST', 'Position', [200, 155, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(getStepVal(), 0));
        uicontrol('Style', 'pushbutton', 'String', '✔ SALVA E CHIUDI', 'Position', [20, 40, 260, 50], 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.8 0.2], 'Callback', @(~,~) closePanel());
        
        sessionActive = true;
        updatePlot(); 
        while sessionActive && ishandle(figRef) && ishandle(figCtrl)
            pause(0.1);
        end
        try close(figCtrl); catch; end
        try close(figRef); catch; end
    end

    %% =========================================================================
    % [5/5] APPLICAZIONE SHIFT, COLORIZZAZIONE E SALVATAGGIO LAS CON VLR (WKT 2112)
    % =========================================================================
    disp('>>> [5/5] Applicazione georeferenziazione definitiva e salvataggio LAS...');
    deltaLat_final = totalShiftNorth_m / metersPerLat;
    deltaLon_final = totalShiftEast_m / metersPerLon;

    lat_final = lat_orig + deltaLat_final;
    lon_final = lon_orig + deltaLon_final;
    [x_new, y_new] = projfwd(projCRSData, lat_final, lon_final);

    validMask = ~isnan(lat_final) & ~isnan(lon_final);
    normLat_adj = (lat_final - latMin) / (latMax - latMin);
    normLon_adj = (lon_final - lonMin) / (lonMax - lonMin);
    pxPoints = round(normLon_adj * (imgWidth - 1)) + 1;
    pyPoints = round((1 - normLat_adj) * (imgHeight - 1)) + 1;
    inImg = validMask & pxPoints >= 1 & pxPoints <= imgWidth & pyPoints >= 1 & pyPoints <= imgHeight;

    pointColors = uint8(ones(length(lat_final), 3) * 128); 
    validIdxs = find(inImg);
    for k = 1:length(validIdxs)
        idx = validIdxs(k);
        pointColors(idx, :) = satImg(pyPoints(idx), pxPoints(idx), :);
    end

    updatedLocations = [x_new, y_new, ptCloud.Location(:, 3)];
    ptCloudColored = pointCloud(updatedLocations, 'Color', pointColors, 'Intensity', ptCloud.Intensity);

    outputLasFilename = [fileBaseName, '_Georeferenziato.las'];
    outputLasFile = fullfile(currentDir, outputLasFilename);

    try
        writer = lasFileWriter(outputLasFile);
        
        try
            wktRecord = lasReader.readVLR(2112);
            if ~isempty(wktRecord)
                writeVLR(writer, wktRecord);
            end
        catch
        end
        
        writePointCloud(writer, ptCloudColored);
        disp([' >>> [SUCCESSO] File LAS georeferenziato salvato in: ', outputLasFile]);
        
    catch ME
        outputPlyFilename = [fileBaseName, '_Georeferenziato.ply'];
        outputLasFile = fullfile(currentDir, outputPlyFilename);
        pcwrite(ptCloudColored, outputLasFile, 'Encoding', 'binary');
        disp([' >>> [Nota] Salvato in formato PLY. Errore LAS: ', ME.message]);
    end

    %% =========================================================================
    % VISUALIZZAZIONE FINALE: 3D GLOBALE & ISPEZIONE TERRENO
    % =========================================================================
    disp('>>> Generazione finestre finali 3D per ispezione...');
    figure('Name', 'Analisi Georeferenziazione - 3D Globale & Ispezione Terreno', 'Position', [50, 50, 1400, 700]);

    subplot(1, 2, 1);
    pcshow(ptCloudColored);
    title({'Nuvola LiDAR 3D Georeferenziata (Updated)', 'Colorizzata da Mappa Satellitare'}, 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X Proiettata (m)'); ylabel('Y Proiettata (m)'); zlabel('Elevazione Z (m)');
    grid on;

    subplot(1, 2, 2);
    groundMask = ~anomalyIdx; 
    groundPtsXYZ = ptCloud.Location(groundMask, :);
    scatter3(groundPtsXYZ(:,1), groundPtsXYZ(:,2), groundPtsXYZ(:,3), 12, groundPtsXYZ(:,3), 'filled');
    colormap(parula);
    colorbar;
    title({'Ispezione 3D della Superficie Terreno', '(Controllo continuità e anomalie altimetriche)'}, 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Quota Z (m)');
    grid on;
    box on;
    axis equal;
    view(3); 

    disp('>>> [Pipeline Completata con Successo!]');
end

%% =========================================================================
% FUNZIONI DI SUPPORTO INTERATTIVO E SEGMENTAZIONE (Nidificate o Locali)
% =========================================================================
function stepVal = getStepVal()
    global stepPopup;
    vals = [0.1, 0.5, 1, 5, 10];
    stepVal = vals(stepPopup.Value);
end

function moveCloud(dEast, dNorth)
    global totalShiftEast_m totalShiftNorth_m;
    totalShiftEast_m = totalShiftEast_m + dEast;
    totalShiftNorth_m = totalShiftNorth_m + dNorth;
    updatePlot();
end

function updatePlot()
    global totalShiftEast_m totalShiftNorth_m spanX_meters spanY_meters imgWidth imgHeight lidarCompositeRGB satImg figRef;
    pxShift = round((totalShiftEast_m / spanX_meters) * imgWidth);
    pyShift = round(-(totalShiftNorth_m / spanY_meters) * imgHeight);
    shiftedLiDAR = imtranslate(lidarCompositeRGB, [pxShift, pyShift], 'FillValues', 0);
    if ishandle(figRef)
        set(0, 'CurrentFigure', figRef);
        clf(figRef);
        imshowpair(satImg, shiftedLiDAR, 'blend');
        title(sprintf('SOVRAPPOSIZIONE INTERATTIVA MANUALE\nShift: E = %.2f m | N = %.2f m', totalShiftEast_m, totalShiftNorth_m));
        drawnow;
    end
end

function closePanel()
    global sessionActive;
    sessionActive = false;
    disp('>>> [Confermato] Allineamento interattivo salvato.');
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