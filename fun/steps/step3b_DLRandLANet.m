function [ptCloudRoad] = step3b_DLRandLANe(ptCloudClean)
    fprintf('\n[TEST 2] Avvio Deep Learning 3D + Filtraggio Rigoroso (Solo Ground)...\n');
    tic;
    
    try
        % 1. DOWNSAMPLING PER RANDLA-NET
        fprintf('  -> Downsampling della nuvola per RandLA-Net (gridStep = 0.3m)...\n');
        gridStep = 0.3; 
        ptCloudDown = pcdownsample(ptCloudClean, 'gridNearest', gridStep);
        
        if ptCloudDown.Count == 0
            ptCloudDown = ptCloudClean;
        end
        
        % 2. CARICAMENTO MODELLO E INFERENCE
        fprintf('  -> Caricamento del segmentatore RandLA-Net ("dales") ed esecuzione...\n');
        segmenter = randlanet("dales");
        labelsDown = segmentObjects(segmenter, ptCloudDown);
        
        % 3. ESTRAZIONE MASCHERE (Ground vs Vegetazione vs Altro)
        if iscategorical(labelsDown)
            cats = categories(labelsDown);
            fprintf('  -> Classi rilevate dal modello: %s\n', strjoin(cats, ', '));
            
            groundMaskDown = (labelsDown == "Ground") | (labelsDown == "Terrain") | (labelsDown == "Road");
            vegMaskDown    = (labelsDown == "Vegetation") | (labelsDown == "Tree") | (labelsDown == "Bush");
            
            if ~any(groundMaskDown)
                groundMaskDown = (labelsDown == cats{1}); 
            end
            if ~any(vegMaskDown)
                vegMaskDown = false(size(labelsDown));
            end
        else
            groundMaskDown = (labelsDown == 1);
            vegMaskDown    = (labelsDown == 2);
        end
        
        % 4. RIPROIEZIONE K-NN A PIENA RISOLUZIONE
        fprintf('  -> Riproiezione k-NN a piena risoluzione su ptCloudClean...\n');
        locsFull = double(ptCloudClean.Location);
        locsDown = double(ptCloudDown.Location);
        
        nearestIdx = knnsearch(locsDown, locsFull, 'K', 1);
        
        groundMaskFull = groundMaskDown(nearestIdx);
        vegMaskFull    = vegMaskDown(nearestIdx);
        otherMaskFull  = ~(groundMaskFull | vegMaskFull); 
        
        % -----------------------------------------------------------------
        % FIGURA 1: VISUALIZZAZIONE 3D SEMANTICA (Silenziosa: Visible = off)
        % -----------------------------------------------------------------
        colors = zeros(ptCloudClean.Count, 3);
        colors(groundMaskFull, :) = repmat([0.2, 0.7, 0.3], sum(groundMaskFull), 1); 
        colors(vegMaskFull, :)    = repmat([0.8, 0.2, 0.2], sum(vegMaskFull), 1);    
        colors(otherMaskFull, :)  = repmat([0.9, 0.6, 0.1], sum(otherMaskFull), 1);  
        
        ptCloudColored = pointCloud(locsFull, 'Color', colors);
        
        fig3D = figure('Name', 'DL Classification 3D', 'Color', 'w', 'Position', [50, 50, 1000, 700], 'Visible', 'off');
        pcshow(ptCloudColored);
        title('Classificazione DL: Verde = Strada, Rosso/Arancione = Scarti');
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        try close(fig3D); catch; end
        
        % -----------------------------------------------------------------
        % FIGURA 2: DIAGNOSTICA 2D DEL PROFILO (Silenziosa: Visible = off)
        % -----------------------------------------------------------------
        xFull = locsFull(:, 1);
        zFull = locsFull(:, 3);
        figDiag = figure('Name', 'Diagnostica DL: Profilo Longitudinale', 'Color', 'w', 'Position', [100, 100, 1200, 600], 'Visible', 'off');
        hold on;
        
        scartatiIndices = find(vegMaskFull | otherMaskFull);
        if ~isempty(scartatiIndices)
            plot(xFull(scartatiIndices), zFull(scartatiIndices), 'r.', 'MarkerSize', 3);
        end
        
        groundIndices = find(groundMaskFull);
        if ~isempty(groundIndices)
            plot(xFull(groundIndices), zFull(groundIndices), 'k.', 'MarkerSize', 4);
        end
        grid on; box on; hold off;
        try close(figDiag); catch; end
        
        % 6. ESTRAZIONE ESCLUSIVA DELLA STRADA
        if isempty(groundIndices)
            error('Nessun punto stradale identificato dopo la riproiezione.');
        end
        ptCloudRawDL = select(ptCloudClean, groundIndices);
        
        % 7. PULIZIA AVANZATA DI ULTERIORI RESIDUI/MACCHIETTE (Post-DL)
        fprintf('  -> Rimozione macchiette e cluster isolati residui...\n');
        ptCloudRoad = pcdenoise(ptCloudRawDL, 'NumNeighbors', 20, 'Threshold', 1.0);
        
        [labelsCluster, numClusters] = pcsegdist(ptCloudRoad, 0.5);
        if numClusters > 1
            clusterSizes = histcounts(labelsCluster, 1:(numClusters+1));
            [~, largestClusterIdx] = max(clusterSizes);
            roadIndicesClean = find(labelsCluster == largestClusterIdx);
            ptCloudRoad = select(ptCloudRoad, roadIndicesClean);
            fprintf('     * Rimossi cluster spuri. Rimane solo il corpo stradale principale (%d punti).\n', ptCloudRoad.Count);
        end
        
        % -----------------------------------------------------------------
        % FIGURA 3: CONFRONTO FINALE (Silenziosa: Visible = off)
        % -----------------------------------------------------------------
        figClean = figure('Name', 'Confronto Nuvola Stradale', 'Color', 'w', 'Position', [150, 150, 1400, 600], 'Visible', 'off');
        subplot(1, 2, 1);
        pcshow(ptCloudRawDL.Location, [0.8, 0.4, 0.2], 'MarkerSize', 2);
        grid on; view(3);
        subplot(1, 2, 2);
        pcshow(ptCloudRoad.Location, [0.2, 0.7, 0.3], 'MarkerSize', 2);
        grid on; view(3);
        try close(figClean); catch; end
        
        fprintf('  -> Filtraggio completato: ora la nuvola contiene SOLO la strada pura!\n');
        
    catch ME
        warning('Deep Learning non eseguibile (%s). Attivo fallback geometrico SMRF.', ME.message);
        try
            [~, ~, ptCloudRoad] = segmentGroundSMRF(ptCloudClean, ...
                'MaxWindowRadius', 5, 'ElevationThreshold', 0.2, 'SlopeThreshold', 0.2);
        catch
            ptCloudRoad = ptCloudClean;
        end
    end
    
    elapsedTime = toc;
    fprintf('  -> Tempo totale: %.2f secondi. Punti stradali finali: %d\n', elapsedTime, ptCloudRoad.Count);
end