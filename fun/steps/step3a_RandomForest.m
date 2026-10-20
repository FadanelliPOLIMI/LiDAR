function [ptCloudRoad, model] = step3a_RandomForest(ptCloudClean, modelInput)
% TEST_APPROACH1_RANDOMFOREST - Pipeline ML Classica PARALLELIZZATA

    fprintf('\n============================================\n');
    fprintf('MACHINE LEARNING: Random Forest (Parallelo)\n');
    fprintf('============================================\n');
    
    tic;
    
    % --- FASE 1: GESTIONE / ADDESTRAMENTO DEL MODELLO ---
    if nargin < 2 || isempty(modelInput)
        modelPath = 'trained_rf.mat';
        if exist(modelPath, 'file')
            fprintf('  -> Caricamento modello Random Forest pre-esistente (%s)...\n', modelPath);
            savedData = load(modelPath, 'model');
            model = savedData.model;
        else
            fprintf('  -> Nessun modello trovato. Avvio addestramento automatico (Training)...\n');
            model = trainRoadClassifier(ptCloudClean);
            save(modelPath, 'model');
            fprintf('  -> Modello salvato con successo in: %s\n', modelPath);
        end
    else
        model = modelInput;
    end

    % --- FASE 2: ESTRAZIONE FEATURE GEOMETRICHE (PARALLELA) ---
    fprintf('  -> Estrazione feature geometriche con supporto multi-core...\n');
    k = 30;
    normals = pcnormals(ptCloudClean, k);
    verticality = abs(normals(:,3));
    nx = abs(normals(:,1));
    ny = abs(normals(:,2));
    nz = normals(:,3);
    
    locations = ptCloudClean.Location;
    numPoints = ptCloudClean.Count;
    
    % Calcolo vicini con knnsearch (ottimizzato)
    idx = knnsearch(locations(:,1:2), locations(:,1:2), 'K', 15);
    
    % CALCOLO RUGOSITÀ PARALLELIZZATO CON PARFOR
    roughness = zeros(numPoints, 1);
    parfor i = 1:numPoints
        neighborZ = locations(idx(i,:), 3);
        roughness(i) = std(neighborZ);
    end
    
    X_features_test = [verticality, nx, ny, nz, roughness];

    % --- FASE 3: CLASSIFICAZIONE CON RANDOM FOREST ---
    fprintf('  -> Esecuzione predizione con Random Forest...\n');
    
    % Abilita l'uso di tutti i core disponibili per la predizione di Ensemble
    % Predizione standard (senza l'argomento 'Options' che genera l'errore)
    predictedLabels = predict(model, X_features_test);

    isRoad = (predictedLabels == true);
    ptCloudRoad = select(ptCloudClean, isRoad);
    
    elapsedTime = toc;
    fprintf('  -> Filtraggio ML parallelo completato in %.2f secondi. Punti stradali: %d\n', ...
        elapsedTime, ptCloudRoad.Count);
    fprintf('============================================\n');
end


%% =========================================================================
% SUB-FUNZIONE 1: ADDESTRAMENTO DEL MODELLO (Training Parallelizzato)
% =========================================================================
function [model] = trainRoadClassifier(ptCloudClean)
    fprintf('--- INIZIO ADDESTRAMENTO PARALLELO (Random Forest) ---\n');
    
    k = 30; 
    normals = pcnormals(ptCloudClean, k);
    verticality = abs(normals(:,3));
    nx = abs(normals(:,1));
    ny = abs(normals(:,2));
    nz = normals(:,3);
    
    locations = ptCloudClean.Location;
    numPoints = ptCloudClean.Count;
    
    idx = knnsearch(locations(:,1:2), locations(:,1:2), 'K', 30);
    
    % Rugosità in parallelo anche in fase di training
    roughness = zeros(numPoints, 1);
    parfor i = 1:numPoints
        neighborZ = locations(idx(i,:), 3);
        roughness(i) = std(neighborZ); 
    end
    
    X_features = [verticality, nx, ny, nz, roughness];

    fprintf('  -> Generazione etichette automatiche con SMRF...\n');
    ptCloudRoadOnly = local_extractGeometricRoad_forTraining(ptCloudClean);
    
    [~, locIndices] = intersect(ptCloudClean.Location, ptCloudRoadOnly.Location, 'rows');
    Y_labels = false(numPoints, 1);
    Y_labels(locIndices) = true;
    
    fprintf('  -> Addestramento Random Forest (Ensemble con 30 alberi in parallelo)...\n');
    
    % Configura l'addestramento dell'Ensemble affinché usi tutti i core della CPU
    options = statset('UseParallel', true);
    model = fitcensemble(X_features, Y_labels, ...
        'Method', 'Bag', ...
        'NumLearningCycles', 30, ...
        'Options', options);
        
    fprintf('  -> Modello addestrato in parallelo con successo!\n');
end


%% =========================================================================
% SUB-FUNZIONE 2: INSEGNANTE SMRF PER LE ETICHETTE DI TRAINING
% =========================================================================
function [ptCloudRoadOnly] = local_extractGeometricRoad_forTraining(ptCloudInput)
    try
        [~, ~, groundPtCloud] = segmentGroundSMRF(ptCloudInput, ...
            'MaxWindowRadius', 5, ...
            'ElevationThreshold', 0.2, ...
            'SlopeThreshold', 0.2);
    catch
        [~, ~, groundPtCloud] = segmentGroundSMRF(ptCloudInput);
    end
    
    if groundPtCloud.Count == 0
        ptCloudRoadOnly = ptCloudInput;
        return;
    end
    
    [labels, numClusters] = pcsegdist(groundPtCloud, 0.5);
    if numClusters == 0
        ptCloudRoadOnly = groundPtCloud;
    else
        clusterCounts = histcounts(labels, 1:(numClusters+1));
        [~, largestClusterIdx] = max(clusterCounts);
        ptCloudRoadOnly = select(groundPtCloud, labels == largestClusterIdx);
    end
end