function [X_final, Y_final, Z_final, rmseVal, maxErrorVal, ptCloudFinal] = step2b_adaptiveVoxelCheck(X_orig, Y_orig, Z_orig, config, ptCloudClean, enablePlot)
% STEP2B_ADAPTIVEVOXELCHECK - Controllo qualità geometrico avanzato (Spaziatura + RMSE + MaxError)
% Preserva integralmente gli attributi Color (RGB) e Intensity.

    fprintf('\n============================================\n');
    fprintf('CONTROLLO QUALITÀ ADAPTIVO (Spaziatura & Voxel)\n');
    fprintf('============================================\n');
    
    % --- 1. Gestione Parametri di Default Sicuri ---
    if nargin < 4 || isempty(config), config = struct(); end
    if ~isfield(config, 'voxelSize'), config.voxelSize = 0.05; end
    if ~isfield(config, 'rmseThreshold'), config.rmseThreshold = 0.020; end % 20 mm
    if ~isfield(config, 'maxErrorThreshold'), config.maxErrorThreshold = 0.015; end % 15 mm
    if nargin < 5, ptCloudClean = []; end
    if nargin < 6 || isempty(enablePlot), enablePlot = false; end

    % --- 2. Costruzione Sicura di ptCloudAOI con Color e Intensity ---
    ptCloudAOI = pointCloud([X_orig, Y_orig, Z_orig]);
    
    % Copia/Propagazione attributi da ptCloudClean
    if ~isempty(ptCloudClean)
        % Gestione Colore
        if isprop(ptCloudClean, 'Color') && ~isempty(ptCloudClean.Color) && size(ptCloudClean.Color, 1) == length(X_orig)
            ptCloudAOI.Color = ptCloudClean.Color;
        end
        % Gestione Intensità
        if isprop(ptCloudClean, 'Intensity') && ~isempty(ptCloudClean.Intensity) && size(ptCloudClean.Intensity, 1) == length(X_orig)
            ptCloudAOI.Intensity = ptCloudClean.Intensity;
        end
    end

    % --- 3. Controllo Intelligente della Spaziatura (Anti-Rada) ---
    minX = min(X_orig); maxX = max(X_orig);
    minY = min(Y_orig); maxY = max(Y_orig);
    
    aoiArea = max(1e-6, (maxX - minX) * (maxY - minY));
    pointDensity = length(X_orig) / aoiArea;        % [punti / m^2]
    avgSpacing = 1 / sqrt(pointDensity);             % [m]
    
    fprintf('  -> Area AOI           : %.2f m^2\n', aoiArea);
    fprintf('  -> Densità stimata    : %.2f pt/m^2\n', pointDensity);
    fprintf('  -> Spaziatura media   : %.4f m (Voxel target: %.2f m)\n', avgSpacing, config.voxelSize);
    
    % Se la nuvola è già rada, salto il downsampling e mantengo gli attributi
    if avgSpacing >= config.voxelSize
        fprintf('\n[LOGICA ADATTIVA] Esito: NUVOLA RADA RILEVATA.\n');
        fprintf('-> Salto il downsampling per evitare degradazione. Propago i dati integrali.\n');
        
        X_final = X_orig; Y_final = Y_orig; Z_final = Z_orig;
        rmseVal = 0.0; maxErrorVal = 0.0;
        ptCloudFinal = ptCloudAOI; % Preserva Location, Color e Intensity
        
        if enablePlot
            visualizeDenoising(ptCloudFinal, ptCloudFinal);
        end
        fprintf('============================================\n');
        return;
    end

    % --- 4. Downsampling Sicuro (pcdownsample media automaticamente Color e Intensity) ---
    ptCloudDown = pcdownsample(ptCloudAOI, 'gridAverage', config.voxelSize);
    
    X_down = double(ptCloudDown.Location(:,1)); 
    Y_down = double(ptCloudDown.Location(:,2)); 
    Z_down = double(ptCloudDown.Location(:,3));

    % --- 5. Definizione Griglia e Clamping Indici ---
    minX_f = floor(min(X_orig)); maxX_f = ceil(max(X_orig));
    minY_f = floor(min(Y_orig)); maxY_f = ceil(max(Y_orig));
    cellSize = max(0.001, config.voxelSize); 
    
    ncols = max(1, ceil((maxX_f - minX_f) / cellSize));
    nrows = max(1, ceil((maxY_f - minY_f) / cellSize));
    
    colOrig = min(ncols, max(1, floor((X_orig - minX_f) / cellSize) + 1));
    rowOrig = min(nrows, max(1, floor((Y_orig - minY_f) / cellSize) + 1));
    gridOrig = accumarray([rowOrig, colOrig], Z_orig, [nrows, ncols], @mean, NaN);
    
    colDown = min(ncols, max(1, floor((X_down - minX_f) / cellSize) + 1));
    rowDown = min(nrows, max(1, floor((Y_down - minY_f) / cellSize) + 1));
    gridDown = accumarray([rowDown, colDown], Z_down, [nrows, ncols], @mean, NaN);

    % --- 6. Calcolo Scarti Robusto ---
    validCells = ~isnan(gridOrig) & ~isnan(gridDown);
    
    if any(validCells(:))
        deltaZ = abs(gridOrig(validCells) - gridDown(validCells));
        deltaZValid = deltaZ(isfinite(deltaZ));
        
        if ~isempty(deltaZValid)
            meanBias    = mean(deltaZValid);
            maxErrorVal = double(max(deltaZValid));
            rmseVal     = double(sqrt(mean(deltaZValid.^2)));
        else
            rmseVal = 0.0; maxErrorVal = 0.0; meanBias = 0.0;
        end
        
        fprintf('  -> Errore medio (Bias)     : %.4f mm\n', meanBias * 1000);
        fprintf('  -> Errore massimo di quota : %.4f mm\n', maxErrorVal * 1000);
        fprintf('  -> RMSE geometrico         : %.4f mm\n', rmseVal * 1000);
    else
        rmseVal = 0.0; maxErrorVal = 0.0;
        warning('Nessuna sovrapposizione tra le celle delle due griglie.');
    end

    % --- 7. Decisione Adattiva e Assegnazione Finale ---
    isAccepted = (rmseVal <= config.rmseThreshold) && (maxErrorVal <= config.maxErrorThreshold);
    if isAccepted
        fprintf('\n[LOGICA ADATTIVA] Esito: SUCCESSO.\n');
        fprintf('-> Voxel accettato: superficie omogenea.\n');
        X_final = X_down; Y_final = Y_down; Z_final = Z_down;
        ptCloudFinal = ptCloudDown; % Contiene la media dei colori e dell'intensità nei voxel
    else
        fprintf('\n[LOGICA ADATTIVA] Esito: FALLBACK ATTIVATO.\n');
        fprintf('-> Rilevata forte variazione locale. Mantengo la nuvola Densa Originale.\n');
        X_final = X_orig; Y_final = Y_orig; Z_final = Z_orig;
        ptCloudFinal = ptCloudAOI; % Contiene i colori e intensità originali
    end

    % --- 8. Visualizzazione 3D Comparativa Opzionale ---
    if enablePlot
        fprintf('  -> Generazione grafico comparativo 3D in corso...\n');
        visualizeDenoising(ptCloudAOI, ptCloudFinal);
    end
    fprintf('============================================\n');
end

%% --- FUNZIONE AUSILIARIA INTERNA ---
function visualizeDenoising(ptCloudRaw, ptCloudClean)
    figure('Name', 'Confronto 3D: Nuvola Densa vs Finale Adattiva', 'Color', 'white', 'Position', [100, 100, 1300, 650]);
    
    ax1 = subplot(1, 2, 1);
    pcshow(ptCloudRaw, 'Parent', ax1);
    title(ax1, sprintf('Nuvola DENSA (Originale)\nPunti: %d', ptCloudRaw.Count), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]'); zlabel(ax1, 'Z [m]');
    grid(ax1, 'on'); axis(ax1, 'equal'); view(ax1, 3);
    
    ax2 = subplot(1, 2, 2);
    pcshow(ptCloudClean, 'Parent', ax2);
    title(ax2, sprintf('Nuvola FINALE (Propagata)\nPunti: %d', ptCloudClean.Count), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax2, 'X [m]'); ylabel(ax2, 'Y [m]'); zlabel(ax2, 'Z [m]');
    grid(ax2, 'on'); axis(ax2, 'equal'); view(ax2, 3);
    
    linkprop([ax1, ax2], {'CameraPosition', 'CameraTarget', 'CameraUpVector', 'CameraViewAngle'});
    fprintf('  -> Finestra di confronto 3D aperta con successo (Viste collegate).\n');
end