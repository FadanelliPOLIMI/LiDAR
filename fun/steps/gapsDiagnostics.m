function [ptCloudFilled, addedCount] = step5b_gapsDiagnostics(ptCloudRoadAligned, sliceWidth)
% STEP5_GAPSDIAGNOSTICS - Inpainting geometrico bidirezionale (X-Y)
% Preserva fedelmente i colori RGB originali e l'Intensity senza alterazioni.

    if nargin < 2 || isempty(sliceWidth), sliceWidth = 0.5; end
    
    fprintf('  -> Avvio Inpainting Geometrico Bidirezionale (X e Y)...\n');
    xyz_orig = double(ptCloudRoadAligned.Location);
    
    % --- 1. Estrazione sicura dei Colori RGB originali (senza alterazioni) ---
    try
        colors_orig = ptCloudRoadAligned.Color;
        if isempty(colors_orig) || size(colors_orig, 1) ~= size(xyz_orig, 1)
            colors_orig = uint8(repmat([128, 128, 128], size(xyz_orig, 1), 1));
        end
    catch
        colors_orig = uint8(repmat([128, 128, 128], size(xyz_orig, 1), 1));
    end
    
    % --- 2. Estrazione sicura dell'Intensity originale ---
    hasIntensity = false;
    intensity_orig = zeros(size(xyz_orig, 1), 1);
    try
        if isprop(ptCloudRoadAligned, 'Intensity') && ~isempty(ptCloudRoadAligned.Intensity)
            tempI = ptCloudRoadAligned.Intensity;
            if length(tempI) == size(xyz_orig, 1)
                intensity_orig = tempI;
                hasIntensity = true;
            end
        end
    catch
    end
    
    % =========================================================================
    % PASSAGGIO 1: INPAINTING LONGITUDINALE (Fette lungo X, interpolazione su Y)
    % =========================================================================
    [ptCloudStep1, ~] = runInpaintingPass(xyz_orig, colors_orig, intensity_orig, sliceWidth, 'X');
    
    % =========================================================================
    % PASSAGGIO 2: INPAINTING TRASVERSALE (Fette lungo Y, interpolazione su X)
    % =========================================================================
    xyz_s1 = double(ptCloudStep1.Location);
    col_s1 = ptCloudStep1.Color;
    int_s1 = ptCloudStep1.Intensity; if ~hasIntensity, int_s1 = zeros(size(xyz_s1,1),1); end
    
    [ptCloudFilled, addedCount] = runInpaintingPass(xyz_s1, col_s1, int_s1, sliceWidth, 'Y');
    
    fprintf('     * Inpainting Bidirezionale Completato. Originali: %d -> Totali: %d (Aggiunti: %d)\n', ...
        size(xyz_orig, 1), ptCloudFilled.Count, addedCount);
    
    % --- DIAGNOSTICA VISIVA ---
    figure('Name', 'Diagnostica Inpainting Bidirezionale', 'Color', 'w', 'Position', [100, 100, 1400, 600]);
    ax1 = subplot(1, 2, 1);
    pcshow(ptCloudRoadAligned, 'Parent', ax1);
    title(sprintf('Originale (Con Vuoti)\nPunti: %d', size(xyz_orig, 1)), 'FontSize', 11, 'FontWeight', 'bold');
    grid on; axis equal; view(3);
    
    ax2 = subplot(1, 2, 2);
    pcshow(ptCloudFilled, 'Parent', ax2);
    title(sprintf('Post-Inpainting Bidirezionale\nPunti Totali: %d (+%d)', ptCloudFilled.Count, addedCount), 'FontSize', 11, 'FontWeight', 'bold');
    grid on; axis equal; view(3);
    drawnow;
    fprintf('>>> Finestra di controllo inpainting aperta con successo.\n');
end

%% --- SUB-ROUTINE PER SINGOLO PASSAGGIO DI INPAINTING ---
%% --- SUB-ROUTINE PER SINGOLO PASSAGGIO DI INPAINTING ---
function [ptCloudOut, synthMaskCount] = runInpaintingPass(xyz, colors, intensity, sliceWidth, direction)
x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

if upper(direction) == 'X'
    coordMain = x; coordSec = y; 
else
    coordMain = y; coordSec = x; 
end

cMin = min(coordMain); cMax = max(coordMain);
cEdges = cMin : sliceWidth : cMax;

filledPoints = [];
filledColors = [];
filledIntensity = [];
syntheticMask = [];

for i = 1:length(cEdges)-1
    cStart = cEdges(i);
    cEnd = cEdges(i+1);

    mask = (coordMain >= cStart) & (coordMain < cEnd);
    if sum(mask) < 10, continue; end

    sMain = coordMain(mask);
    sSec = coordSec(mask);
    sZ = z(mask);
    sC = colors(mask, :);
    sI = intensity(mask);

    [sSec_sorted, sortIdx] = sort(sSec);
    sZ_sorted = sZ(sortIdx);
    sMain_sorted = sMain(sortIdx);
    sC_sorted = sC(sortIdx, :);
    sI_sorted = sI(sortIdx);

    [sSec_unique, ~, ic] = unique(sSec_sorted);

    if length(sSec_unique) > 2
        sZ_unique = accumarray(ic, sZ_sorted, [], @mean);

        % Gestione unicità per i colori e l'intensità associati alle quote uniche
        sC_unique = zeros(length(sSec_unique), size(sC_sorted, 2));
        for c_idx = 1:size(sC_sorted, 2)
            sC_unique(:, c_idx) = accumarray(ic, double(sC_sorted(:, c_idx)), [], @(v) round(mean(v)));
        end
        sI_unique = accumarray(ic, double(sI_sorted), [], @mean);

        secMin = min(sSec_unique);
        secMax = max(sSec_unique);
        secQuery = secMin : 0.05 : secMax; 

        zQuery = interp1(sSec_unique, sZ_unique, secQuery, 'linear', 'extrap');
        mainQuery = mean(sMain_sorted) * ones(size(secQuery));

        % --- CORRETTO: Uso di 'linear' ed 'extrap' senza conflitti ---
        cQuery = zeros(length(secQuery), size(sC_sorted, 2));
        for c_idx = 1:size(sC_sorted, 2)
            cQuery(:, c_idx) = interp1(sSec_unique, double(sC_unique(:, c_idx)), secQuery, 'linear', 'extrap');
        end
        cQuery = uint8(max(0, min(255, cQuery))); % Clamp sicuro tra 0 e 255 per il RGB

        iQuery = interp1(sSec_unique, sI_unique, secQuery, 'linear', 'extrap');

        if upper(direction) == 'X'
            newXYZ = [mainQuery', secQuery', zQuery'];
        else
            newXYZ = [secQuery', mainQuery', zQuery'];
        end

        filledPoints = [filledPoints; xyz(mask, :); newXYZ];
        filledColors = [filledColors; sC; cQuery];
        filledIntensity = [filledIntensity; sI; iQuery'];

        syntheticMask = [syntheticMask; false(length(sMain), 1); true(length(secQuery), 1)];
    else
        filledPoints = [filledPoints; xyz(mask, :)];
        filledColors = [filledColors; sC];
        filledIntensity = [filledIntensity; sI];
        syntheticMask = [syntheticMask; false(length(sMain), 1)];
    end
end

ptCloudOut = pointCloud(filledPoints, 'Color', filledColors, 'Intensity', filledIntensity);
synthMaskCount = sum(syntheticMask);
end