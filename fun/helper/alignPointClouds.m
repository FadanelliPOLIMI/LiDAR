function [totalShiftEast_m, totalShiftNorth_m] = alignPointClouds(ctx, config)
    % ALIGNPOINTCLOUDS Runs DL -> HOG -> imregcorr with Manual Interactive Fallback.

    global totalShiftEast_m totalShiftNorth_m spanX_meters spanY_meters imgWidth imgHeight lidarCompositeRGB satImg figRef stepPopup sessionActive;
    
    spanX_meters = ctx.spanX_meters;
    spanY_meters = ctx.spanY_meters;
    imgWidth = ctx.imgWidth;
    imgHeight = ctx.imgHeight;
    lidarCompositeRGB = ctx.lidarCompositeRGB;
    satImg = ctx.satImg;
    satGray = ctx.satGray;
    lidarGray = ctx.lidarGray;

    alignmentAccepted = false;
    totalShiftEast_m = 0; 
    totalShiftNorth_m = 0; 

    disp('>>> [POINTS ALIGNMENT] Starting automated georeferencing sequence...');
    
    % --- STEP A: DEEP LEARNING (ResNet-50) ---
    if ~alignmentAccepted
        disp(' -> Attempt 1/3: Deep Learning (ResNet-50)...');
        try
            net = resnet50;
            imgRGB_LiDAR = cat(3, lidarCompositeRGB(:,:,1), lidarCompositeRGB(:,:,1), lidarCompositeRGB(:,:,1));
            imgRGB_Sat   = cat(3, satGray, satGray, satGray);
            featuresLiDAR_CNN = activations(net, imgRGB_LiDAR, 'activation_40_relu', 'OutputAs', 'channels');
            featuresSat_CNN   = activations(net, imgRGB_Sat,   'activation_40_relu', 'OutputAs', 'channels');
            featMapLiDAR = mean(featuresLiDAR_CNN, 3);
            featMapSat   = mean(featuresSat_CNN, 3);
            
            tformDeep = imregcorr(featMapLiDAR, featMapSat, 'translation');
            shiftE_deep = (tformDeep.T(3, 1) / size(featMapSat, 2)) * spanX_meters;
            shiftN_deep = -(tformDeep.T(3, 2) / size(featMapSat, 1)) * spanY_meters;
            
            figPrev = figure('Name', 'Verification - Deep Learning', 'Position', [0, 0, 800, 700]);
            movegui(figPrev, 'center'); 
            pxPrev = round((shiftE_deep / spanX_meters) * imgWidth);
            pyPrev = round(-(shiftN_deep / spanY_meters) * imgHeight);
            prevLiDAR = imtranslate(lidarCompositeRGB, [pxPrev, pyPrev], 'FillValues', 0);
            imshowpair(satImg, prevLiDAR, 'blend');
            title(sprintf('Deep Learning Verification\nEast: %.2f m | North: %.2f m', shiftE_deep, shiftN_deep));
            drawnow;
            
            choiceDeep = questdlg('Is the Deep Learning alignment correct?', 'DL Confirmation', 'Yes', 'No', 'Yes');
            try close(figPrev); catch; end
            
            if strcmp(choiceDeep, 'Yes')
                totalShiftEast_m = shiftE_deep;
                totalShiftNorth_m = shiftN_deep;
                alignmentAccepted = true;
                disp(' >>> [OK] Deep Learning accepted.');
            end
        catch ME
            disp([' >>> [DL Warning]: ', ME.message]);
        end
    end

    % --- STEP B: FEATURE MATCHING (Adaptive HOG) ---
    if ~alignmentAccepted
        disp(' -> Attempt 2/3: Feature Matching (Adaptive HOG)...');
        try
            intensityImg = lidarCompositeRGB(:,:,1);
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
                
                figPrev = figure('Name', 'Verification - Adaptive HOG', 'Position', [0, 0, 800, 700]);
                movegui(figPrev, 'center'); 
                pxPrev = round((shiftE_feat / spanX_meters) * imgWidth);
                pyPrev = round(-(shiftN_feat / spanY_meters) * imgHeight);
                prevLiDAR = imtranslate(lidarCompositeRGB, [pxPrev, pyPrev], 'FillValues', 0);
                imshowpair(satImg, prevLiDAR, 'blend');
                title(sprintf('HOG Verification (Inliers: %d)\nEast: %.2f m | North: %.2f m', sum(inliers), shiftE_feat, shiftN_feat));
                drawnow;
                
                choiceFeat = questdlg('Is the HOG alignment correct?', 'HOG Confirmation', 'Yes', 'No', 'Yes');
                try close(figPrev); catch; end
                    
                if strcmp(choiceFeat, 'Yes')
                    totalShiftEast_m = shiftE_feat;
                    totalShiftNorth_m = shiftN_feat;
                    alignmentAccepted = true;
                    disp(' >>> [OK] HOG accepted.');
                end
            end
        catch ME
            disp([' >>> [HOG Warning]: ', ME.message]);
        end
    end

    % --- STEP C: IMREGCORR ---
    if ~alignmentAccepted
        disp(' -> Attempt 3/3: Direct Raster Correlation (imregcorr)...');
        try
            tformReg = imregcorr(lidarGray, satGray, 'translation');
            shiftE_reg = (tformReg.T(3, 1) / imgWidth) * spanX_meters;
            shiftN_reg = -(tformReg.T(3, 2) / imgHeight) * spanY_meters; 
            
            figPrev = figure('Name', 'Verification - imregcorr', 'Position', [0, 0, 800, 700]);
            movegui(figPrev, 'center'); 
            pxPrev = round((shiftE_reg / spanX_meters) * imgWidth);
            pyPrev = round(-(shiftN_reg / spanY_meters) * imgHeight);
            prevLiDAR = imtranslate(lidarCompositeRGB, [pxPrev, pyPrev], 'FillValues', 0);
            imshowpair(satImg, prevLiDAR, 'blend');
            title(sprintf('imregcorr Verification\nEast: %.2f m | North: %.2f m', shiftE_reg, shiftN_reg));
            drawnow;
            
            choiceReg = questdlg('Is the imregcorr alignment correct?', 'imregcorr Confirmation', 'Yes', 'No', 'Yes');
            try close(figPrev); catch; end
                
            if strcmp(choiceReg, 'Yes')
                totalShiftEast_m = shiftE_reg;
                totalShiftNorth_m = shiftN_reg;
                alignmentAccepted = true;
                disp(' >>> [OK] imregcorr accepted.');
            end
        catch ME
            disp([' >>> [imregcorr Warning]: ', ME.message]);
        end
    end

    % --- STEP D: MANUAL INTERACTIVE FALLBACK (GUI) ---
    if ~alignmentAccepted
        disp(' >>> All automated methods rejected. Opening manual control panel...');
        
        figRef = figure('Name', 'Interactive LiDAR on Satellite Overlay Control', 'Position', [50, 50, 900, 800]);
        figCtrl = figure('Name', 'Shift Control Panel', 'Position', [980, 400, 300, 350], 'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');
        
        uicontrol('Style', 'text', 'Position', [20, 300, 260, 30], 'String', 'Select Shift Step Size:', 'FontSize', 10, 'FontWeight', 'bold');
        stepPopup = uicontrol('Style', 'popupmenu', 'Position', [20, 270, 260, 30], 'String', {'0.1 Meters', '0.5 Meters', '1 Meter', '5 Meters', '10 Meters'}, 'Value', 3);
        
        uicontrol('Style', 'pushbutton', 'String', '▲ NORTH', 'Position', [110, 200, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(0, getStepVal()));
        uicontrol('Style', 'pushbutton', 'String', '▼ SOUTH', 'Position', [110, 110, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(0, -getStepVal()));
        uicontrol('Style', 'pushbutton', 'String', '◄ WEST', 'Position', [20, 155, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(-getStepVal(), 0));
        uicontrol('Style', 'pushbutton', 'String', '► EAST', 'Position', [200, 155, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(getStepVal(), 0));
        
        uicontrol('Style', 'pushbutton', 'String', '✔ SAVE AND CLOSE', 'Position', [20, 40, 260, 50], 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.8 0.2], 'Callback', @(~,~) closePanel());
        
        sessionActive = true;
        updatePlot(); 
        
        while sessionActive && ishandle(figRef) && ishandle(figCtrl)
            pause(0.1);
        end
        
        try close(figCtrl); catch; end
        try close(figRef); catch; end
    end
end

% --- Helper Callbacks for GUI ---
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
        title(sprintf('MANUAL INTERACTIVE OVERLAY\nShift: E = %.2f m | N = %.2f m', totalShiftEast_m, totalShiftNorth_m));
        drawnow;
    end
end

function closePanel()
    global sessionActive;
    sessionActive = false;
    disp('>>> [Confirmed] Interactive alignment saved.');
end