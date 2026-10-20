function run_StreamingLiDARPipeline(sensorStreamSource)
% RUN_STREAMINGLIDARPIPELINE - Pipeline di streaming e apprendimento online per LiDAR

fprintf('=== AVVIO STREAMING & ONLINE ACTIVE LEARNING ===\n');

% 1. Inizializzazione del classificatore incrementale (Apprendimento online)
% Usiamo un modello lineare incrementale per la classificazione Terra / Non-Terra
incrementalModel = incrementalClassifier('ModelType', 'Linear', ...
    'MetricsWindowSize', 500); 

isModelWarm = false;
gridResolution = 0.1;

% Simulazione o lettura del flusso di pacchetti in un ciclo continuo
while hasData(sensorStreamSource) 

    % A. RICEZIONE PACCHETTO DATI (Frame o Mini-Batch LiDAR)
    ptCloudPacket = readNextPacket(sensorStreamSource);
    if ptCloudPacket.Count == 0, continue; end

    xyz = double(ptCloudPacket.Location);
    x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

    % B. PSEUDO-LABELING ISTANTANEO (Euristica geometrica locale sul pacchetto)
    binSize = 0.5;
    xMin = min(x); yMin = min(y);
    xBinIdx = floor((x - xMin) / binSize) + 1;
    yBinIdx = floor((y - yMin) / binSize) + 1;
    validMask = ~isnan(z);
    [~, ~, ic] = unique([xBinIdx(validMask), yBinIdx(validMask)], 'rows');
    zMinBin = accumarray(ic, z(validMask), [], @min);
    heights = z(validMask) - zMinBin(ic);

    % Pseudo-label: 0 = Terra, 1 = Non-Terra
    packetLabels = ones(length(z), 1);
    packetLabels(validMask & (heights < 0.30)) = 0; 

    % Estrazione feature locali (Altezza e Rugosità)
    X_stream = [heights, abs(heights - mean(heights))];
    y_stream = packetLabels(validMask);
    X_stream_valid = X_stream(validMask, :);

    % C. APPRENDIMENTO INCREMENTALE ("Learning by Doing")
    if ~isModelWarm
        % Riscaldamento iniziale del modello con i primi dati
        incrementalModel = fit(incrementalModel, X_stream_valid, y_stream);
        if incrementalModel.IsWarm
            isModelWarm = true;
            fprintf('  -> Modello incrementale "caldo" e operativo nello streaming.\n');
        end
    else
        % Flusso continuo: aggiorna metriche e adatta il modello al volo
        incrementalModel = updateMetricsAndFit(incrementalModel, X_stream_valid, y_stream);
        fi

        % D. CLASSIFICAZIONE ONLINE DEL PACCHETTO
        predictedStreamLabels = predict(incrementalModel, X_stream_valid);
        groundMaskPacket = (predictedStreamLabels == 0);

        % E. ESTRAZIONE PUNTI DI TERRA DAL PACCHETTO CORRENTE
        validIndices = find(validMask);
        groundIndices = validIndices(groundMaskPacket);
        ptCloudGroundPacket = pointCloud(xyz(groundIndices, :));

        % F. AGGIORNAMENTO DINAMICO DELLA SUPERFICIE (Locale)
        if ptCloudGroundPacket.Count > 10
            % Qui aggiorniamo la patch locale OpenCRG associata a queste coordinate (x,y)
            fprintf('  -> Ricevuti %d punti di terra puliti. Aggiornamento patch OpenCRG...\n', ...
                ptCloudGroundPacket.Count);

            % (Integrazione con la griglia globale o salvataggio incrementale per ADAMS)
        end

        % Condizione di stop simulazione (es. fine file o interruzione utente)
        % break; 
    end

    fprintf('=== STREAMING TERMINATO CON SUCCESSO ===\n');
end