function [Xq, Yq, Z_spectral, dominantWavelength] = interpolate_Spectral_FFT(ptCloudGround, config)
    % INTERPOLATE_SPECTRAL_FFT - Advanced 2D Spectral Analysis with Whitened PSD
    % and Strict Footprint Masking. Fully generalized for any terrain wavelength 
    % without hardcoded assumptions.
    %
    % SYNTAX:
    %   [Xq, Yq, Z_spectral, dominantWavelength] = interpolate_Spectral_FFT(ptCloudGround, config)

    % --- 1. CONFIGURATION PARSING ---
    gridResolution   = 0.05; % Recommended 5cm resolution
    enablePlot       = true;  % Default to true for inspection
    maxDistThreshold = 0.25;  % Max distance (m) from real points to keep grid cell valid

    if nargin >= 2 && ~isempty(config)
        if isstruct(config)
            if isfield(config, 'voxelSize') && ~isempty(config.voxelSize)
                gridResolution = config.voxelSize;
            end
            if isfield(config, 'visible') && ~isempty(config.visible)
                enablePlot = config.visible;
            end
            if isfield(config, 'maxDistThreshold') && ~isempty(config.maxDistThreshold)
                maxDistThreshold = config.maxDistThreshold;
            end
        elseif isnumeric(config)
            gridResolution = config;
        end
    end

    % --- 2. EXTRACT COORDINATES ---
    if isa(ptCloudGround, 'pointCloud')
        xyz = double(ptCloudGround.Location);
        xMin = ptCloudGround.XLimits(1); xMax = ptCloudGround.XLimits(2);
        yMin = ptCloudGround.YLimits(1); yMax = ptCloudGround.YLimits(2);
    else
        xyz = double(ptCloudGround);
        xMin = min(xyz(:,1)); xMax = max(xyz(:,1));
        yMin = min(xyz(:,2)); yMax = max(xyz(:,2));
    end
    
    x = xyz(:, 1); y = xyz(:, 2); z = xyz(:, 3);
    
    % --- 3. INITIAL REGULAR GRID RASTERIZATION ---
    xGrid = xMin : gridResolution : xMax;
    yGrid = yMin : gridResolution : yMax;
    [Xq, Yq] = meshgrid(xGrid, yGrid);
    
    F = scatteredInterpolant(x, y, z, 'linear', 'none');
    Z_raw = F(Xq, Yq);
    
    % Clean handling of empty border gaps for FFT math only
    Z_raw_filled = fillmissing(Z_raw, 'linear', 2);
    Z_raw_filled = fillmissing(Z_raw_filled, 'linear', 1);
    Z_raw_filled(isnan(Z_raw_filled)) = mean(z, 'omitnan');
    [rows, cols] = size(Z_raw_filled);

    % --- 4. 2D DETRENDING (Global Slope Removal) ---
    [X_mesh, Y_mesh] = meshgrid(1:cols, 1:rows);
    A_mat = [X_mesh(:), Y_mesh(:), ones(numel(Z_raw_filled), 1)];
    p_plane = A_mat \ Z_raw_filled(:);
    Z_trend = reshape(A_mat * p_plane, rows, cols);
    Z_residue = Z_raw_filled - Z_trend;

    % --- 5. 2D FFT & SPECTRAL WHITENING ---
    F2 = fft2(Z_residue);
    F2_shift = fftshift(F2);
    
    % Raw PSD
    PSD2D = abs(F2_shift).^2;
    
    % Build spatial frequency map (cycles / meter)
    [FX, FY] = meshgrid(linspace(-cols/2, cols/2, cols), linspace(-rows/2, rows/2, rows));
    freqX = FX / (cols * gridResolution);
    freqY = FY / (rows * gridResolution);
    spatialFreqMap = sqrt(freqX.^2 + freqY.^2);
    
    % SPECTRAL WHITENING: Multiply PSD by frequency squared (f^2) to flatten 
    % the natural 1/f^2 terrain drop-off, bringing micro-peaks into view
    PSD_whitened = PSD2D .* (spatialFreqMap.^2);
    
    % --- 6. UNCONSTRAINED PEAK SEARCH (General Case) ---
    [rowCenter, colCenter] = deal(floor(rows/2)+1, floor(cols/2)+1);
    PSD_search = PSD_whitened;
    if rows > 6 && cols > 6
        dcRadius = 3;
        [dxMesh, dyMesh] = meshgrid(-cols/2:cols/2, -rows/2:rows/2);
        dcMask = (sqrt(dxMesh.^2 + dyMesh.^2) <= dcRadius);
        PSD_search(dcMask) = 0; 
    end
    
    % Find global maximum peak in the whitened spectrum without bounds
    [~, maxIdx] = max(PSD_search(:));
    [peakRow, peakCol] = ind2sub([rows, cols], maxIdx);
    
    spatialFreq = spatialFreqMap(peakRow, peakCol);
    if spatialFreq > 0
        dominantWavelength = 1 / spatialFreq;
    else
        dominantWavelength = 2.0; % General safe fallback
    end

    % --- 7. FILTERING & RECONSTRUCTION ---
    targetFreq = 1 / dominantWavelength;
    bandwidth = targetFreq * 0.6; 
    H_band = exp(-((spatialFreqMap - targetFreq).^2) / (2 * (bandwidth^2)));
    
    F2_filtered = F2_shift .* H_band;
    Z_filtered_residue = real(ifft2(ifftshift(F2_filtered)));
    
    % Final reconstruction: Global trend + filtered harmonic residue
    Z_spectral = Z_trend + Z_filtered_residue;

    % --- 8. STRICT FOOTPRINT MASKING (Prevent Hallucinations) ---
    gridPoints = [Xq(:), Yq(:)];
    [~, dists] = knnsearch(xyz(:, 1:2), gridPoints);
    dists = reshape(dists, rows, cols);
    
    footprintMask = (dists <= maxDistThreshold);
    Z_spectral(~footprintMask) = NaN; % Restrict output strictly to real data footprint

    fprintf('[Advanced 2D FFT] Generalized analysis complete. Dominant wavelength detected: %.2f m.\n', dominantWavelength);

    % --- 9. ENHANCED 4-PANEL VISUALIZATION DASHBOARD ---
    if enablePlot
        figure('Name', '2D Spectral Whitening & Generalized Peak Analysis', 'Color', 'w', 'Position', [50, 50, 1600, 700]);
        
        % Panel 1: Spatial Residual
        subplot(2, 2, 1);
        imagesc(xGrid, yGrid, Z_residue);
        set(gca, 'YDir', 'normal'); colormap(gca, jet); colorbar;
        title('1. Detrended Spatial Residual', 'FontWeight', 'bold');
        xlabel('X [m]'); ylabel('Y [m]'); axis equal tight;
        
        % Panel 2: Whitened PSD with Dynamic Peak & Detected Wavelength Ring
        subplot(2, 2, 2);
        imagesc(linspace(-cols/2, cols/2, cols), linspace(-rows/2, rows/2, rows), log(PSD_whitened + 1));
        set(gca, 'YDir', 'normal'); colormap(gca, jet); colorbar;
        hold on;
        plot(peakCol - colCenter, peakRow - rowCenter, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
        
        % Dynamic reference circle based on the *actually detected* wavelength
        theta = linspace(0, 2*pi, 100);
        detectedRadiusBins = spatialFreq * (cols * gridResolution);
        plot(detectedRadiusBins * cos(theta), detectedRadiusBins * sin(theta), 'w--', 'LineWidth', 1.5);
        hold off;
        
        title(sprintf('2. Whitened PSD (Red=Peak @ \\lambda=%.2fm)\nWhite Dashed = Detected \\lambda Ring', dominantWavelength), 'FontWeight', 'bold');
        xlabel('Frequency X (Cycles/m)'); ylabel('Frequency Y (Cycles/m)');
        axis equal tight;
        xlim([-cols/2, cols/2]); ylim([-rows/2, rows/2]);
        
        % Panel 3: 2D Reconstructed Surface (Masked)
        subplot(2, 2, 3);
        imagesc(xGrid, yGrid, Z_spectral);
        set(gca, 'YDir', 'normal'); colormap(gca, jet); colorbar;
        title('3. Reconstructed Surface (Footprint Masked)', 'FontWeight', 'bold');
        xlabel('X [m]'); ylabel('Y [m]'); axis equal tight;
        
    end
end
