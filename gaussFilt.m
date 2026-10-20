function [ptCloudClean, X_grid, Y_grid, Z_grid_clean] = gaussFilt(ptCloudFilled, gridRes, sigmaSmooth)
% STEP6_GENERATECLEANGRID - Restituisce un pointCloud pulito preservando Colori e Intensità.

    if nargin < 2 || isempty(gridRes), gridRes = 0.05; end          
    if nargin < 3 || isempty(sigmaSmooth), sigmaSmooth = 1.0; end 

    fprintf('\n============================================\n');
    fprintf('GENERAZIONE POINT CLOUD PULITA (Anti Scan-Lines)\n');
    fprintf('============================================\n');
    
    xyz = double(ptCloudFilled.Location);
    x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);
    
    % --- 1. Estrazione sicura di Colore e Intensità originali ---
    hasColor = false;
    hasIntensity = false;
    
    try
        if isprop(ptCloudFilled, 'Color') && ~isempty(ptCloudFilled.Color)
            colors_orig = double(ptCloudFilled.Color);
            if size(colors_orig, 1) == length(x), hasColor = true; end
        end
    catch
    end
    
    try
        if isprop(ptCloudFilled, 'Intensity') && ~isempty(ptCloudFilled.Intensity)
            intensity_orig = double(ptCloudFilled.Intensity);
            if length(intensity_orig) == length(x), hasIntensity = true; end
        end
    catch
    end

    % --- 2. Creazione della griglia regolare 2D ---
    xVec = min(x) : gridRes : max(x);
    yVec = min(y) : gridRes : max(y);
    [X_grid, Y_grid] = meshgrid(xVec, yVec);
    
    fprintf('  -> Interpolazione geometrica sulla griglia (Res: %.3f m)...\n', gridRes);
    Z_grid = griddata(x, y, z, X_grid, Y_grid, 'linear');
    
    % Chiusura bordi vuoti per la quota Z
    Z_grid = fillmissing(Z_grid, 'linear', 2);
    Z_grid = fillmissing(Z_grid, 'linear', 1);
    Z_grid(isnan(Z_grid)) = mean(z, 'omitnan');
    
    % --- 3. Filtraggio Bidimensionale per rimuovere le scan-lines ---
    fprintf('  -> Applicazione filtro gaussiano 2D (sigma = %.1f)...\n', sigmaSmooth);
    Z_grid_clean = imgaussfilt(Z_grid, sigmaSmooth);
    
    % --- 4. Ricostruzione della Point Cloud pulita ---
    X_flat = X_grid(:);
    Y_flat = Y_grid(:);
    Z_flat = Z_grid_clean(:);
    
    validIdx = ~isnan(Z_flat);
    cleanPts = [X_flat(validIdx), Y_flat(validIdx), Z_flat(validIdx)];
    
    args = {cleanPts};
    
    % Gestione robusta dei colori interpolati sulla griglia
    if hasColor
        C_flat = zeros(length(X_flat), size(colors_orig, 2));
        for c_idx = 1:size(colors_orig, 2)
            cGrid = griddata(x, y, colors_orig(:, c_idx), X_grid, Y_grid, 'linear');
            % Fallback per eventuali NaN ai bordi usando la media o il più vicino
            cGrid(isnan(cGrid)) = median(colors_orig(:, c_idx), 'omitnan');
            C_flat(:, c_idx) = cGrid(:);
        end
        % Pulizia NaN residui e conversione sicura in uint8
        C_clean = C_flat(validIdx, :);
        C_clean(isnan(C_clean)) = 128; % Grigio di sicurezza se proprio serve
        args = [args, {'Color', uint8(max(0, min(255, C_clean)))}];
    end
    
    % Gestione robusta dell'intensità interpolata sulla griglia
    if hasIntensity
        iGrid = griddata(x, y, intensity_orig, X_grid, Y_grid, 'linear');
        iGrid(isnan(iGrid)) = median(intensity_orig, 'omitnan');
        I_flat = iGrid(:);
        I_clean = I_flat(validIdx);
        I_clean(isnan(I_clean)) = 0;
        args = [args, {'Intensity', I_clean}];
    end
    
    ptCloudClean = pointCloud(args{:});
    
    fprintf('     * Point Cloud pulita generata con successo. Punti totali: %d\n', ptCloudClean.Count);
    
    % --- 5. Visualizzazione Comparativa 3D ---
    figure('Name', 'Confronto Nuvola di Punti: Originale vs Pulita', 'Color', 'w', 'Position', [100, 100, 1400, 600]);
    
    ax1 = subplot(1, 2, 1);
    pcshow(ptCloudFilled, 'Parent', ax1);
    title(ax1, 'Nuvola Post-Inpainting (Con scan-lines)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); grid on; axis equal; view(3);
    
    ax2 = subplot(1, 2, 2);
    pcshow(ptCloudClean, 'Parent', ax2);
    title(ax2, sprintf('Nuvola Pulita (Filtro 2D \\sigma=%.1f)', sigmaSmooth), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); grid on; axis equal; view(3);
    
    linkprop([ax1, ax2], {'CameraPosition', 'CameraTarget', 'CameraUpVector', 'CameraViewAngle'});
    fprintf('>>> Finestra di controllo nuvola pulita aperta con successo.\n');
    fprintf('============================================\n');
end