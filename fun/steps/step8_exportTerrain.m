function Zterrain = step8_exportTerrain(ptCloudFinalROI, config)
    % STEP8_EXPORTTERRAIN - Pure rasterization, slope clamping, and visualization.
    % Bins the pre-processed point cloud into a 2D matrix, applies the 
    % iterative gradient bulldozer to clamp unsafe slopes, and renders inspection plots.
    
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    gridRes = config.voxelSize;
    maxGradLimit = config.maxSafeGradientPct;
    
    % =========================================================
    % 1. NON-DESTRUCTIVE 3D SCAN-LINE REMOVAL (Radius = 0.15m)
    % =========================================================
    fprintf('  -> Applying Local 3D Surface Smoothing (Scan-line removal)...\n');
    locs = single(ptCloudFinalROI.Location);
    numPts = size(locs, 1);
    smoothedLocs = zeros(numPts, 3, 'single');
    
    kdtree = KDTreeSearcher(locs(:, 1:2));
    [idxNeighbors, ~] = rangesearch(kdtree, locs(:, 1:2), single(0.15));
    
    for i = 1:numPts
        nIdx = idxNeighbors{i};
        if length(nIdx) > 3
            smoothedLocs(i, 1:2) = locs(i, 1:2);
            smoothedLocs(i, 3) = mean(locs(nIdx, 3)); 
        else
            smoothedLocs(i, :) = locs(i, :);
        end
    end
    
    args = {smoothedLocs};
    if isprop(ptCloudFinalROI, 'Color') && ~isempty(ptCloudFinalROI.Color), args = [args, {'Color', ptCloudFinalROI.Color}]; end
    if isprop(ptCloudFinalROI, 'Intensity') && ~isempty(ptCloudFinalROI.Intensity), args = [args, {'Intensity', ptCloudFinalROI.Intensity}]; end
    ptCloudClean = pointCloud(args{:});

    % =========================================================
    % 2. PURE GRID BINNING (RASTERIZATION ONLY)
    % =========================================================
    cleanLocs = ptCloudClean.Location;
    X_pts = cleanLocs(:,1); Y_pts = cleanLocs(:,2); Z_pts = cleanLocs(:,3);
    
    minX = floor(min(X_pts)); maxX = ceil(max(X_pts));
    minY = floor(min(Y_pts)); maxY = ceil(max(Y_pts));
    xEdges = minX : gridRes : maxX;
    yEdges = minY : gridRes : maxY;
    ncols = length(xEdges) - 1;
    nrows = length(yEdges) - 1;

    fprintf('  -> Binning clean point cloud to %dx%d matrix...\n', nrows, ncols);
    [~, ~, ~, binX, binY] = histcounts2(X_pts, Y_pts, xEdges, yEdges);
    linIdx = sub2ind([ncols, nrows], binX(binX>0 & binY>0), binY(binX>0 & binY>0));
    ZVector = accumarray(linIdx, Z_pts(binX>0 & binY>0), [ncols * nrows, 1], @mean, single(NaN));
    ZterrainBase = reshape(ZVector, [ncols, nrows])'; 
    Zterrain = flipud(ZterrainBase);

    % Store exact boundary mask of un-scanned outer space
    exteriorNaNs = isnan(Zterrain);

    % =========================================================
    % 3. ITERATIVE GRADIENT BULLDOZER (Clamps slopes > maxGradLimit)
    % =========================================================
    % Temporarily pad outer NaNs so filtering doesn't pull NaNs into valid road edges
    Z_filter = fillmissing(Zterrain, 'nearest');
    
    Z_filter = medfilt2(Z_filter, [3 3], 'symmetric'); % 1-pixel spike filter
    
    maxIterations = 40;
    tol = 0.5;
    iter = 0;
    
    fprintf('  -> Running Iterative Gradient Bulldozer (Target < %.1f%%)...\n', maxGradLimit);
    while iter < maxIterations
        [dZx, dZy] = gradient(Z_filter, gridRes, gridRes);
        currentSlopePct = sqrt(dZx.^2 + dZy.^2) * 100;
        maxCurrentSlope = max(currentSlopePct(:));
        
        if maxCurrentSlope <= (maxGradLimit + tol)
            fprintf('     * Target slope reached: %.1f%% after %d iterations.\n', maxCurrentSlope, iter);
            break;
        end
        
        violators = currentSlopePct > maxGradLimit;
        H_blur = fspecial('average', [5 5]);
        Z_blurred = imfilter(Z_filter, H_blur, 'replicate');
        expandedViolators = imdilate(violators, strel('square', 3));
        Z_filter(expandedViolators) = Z_blurred(expandedViolators);
        
        iter = iter + 1;
    end

    % Restore exact outer NaNs (preserving natural scanned footprint)
    Z_filter(exteriorNaNs) = single(NaN);
    Zterrain = Z_filter;

    % =========================================================
    % 4. TERRAIN HEALTH DIAGNOSTICS
    % =========================================================
    [dZx, dZy] = gradient(Zterrain, gridRes, gridRes);
    slopePct = sqrt(dZx.^2 + dZy.^2) * 100;
    maxSlopePct = max(slopePct(:), [], 'omitnan');
    fprintf('  -> Final Max Gradient: %.1f%% slope\n', maxSlopePct);

    % =========================================================
    % 5. FLUID FULL-ROAD INSPECTOR PLOT (Conditional on config.visible)
    % =========================================================
    if isfield(config, 'visible') && config.visible
        fprintf('\n--------------------------------------------\n');
        fprintf('EXTRA PLOT: Rendering Fluid Full-Road Inspector\n');
        fprintf('--------------------------------------------\n');
        
        [X_full, Y_full] = meshgrid((0:ncols-1)*gridRes, (0:nrows-1)*gridRes);
        figFluid = figure('Name', 'Fluid Road Surface Inspector', 'Color', 'w', 'Position', [100, 100, 1300, 750]);
        
        hFullSurf = surf(X_full, Y_full, Zterrain);
        set(hFullSurf, 'EdgeColor', 'none');
        
        colormap(turbo);
        colorbar;
        shading interp;
        lightangle(-45, 45);
        lighting gouraud;
        material dull;
        
        yStretch = max(1, (ncols*gridRes) / (nrows*gridRes) / 6);
        zStretch = 15; 
        daspect([yStretch, 1, 1/zStretch]); 
        
        axis tight; box on; grid on;
        
        totalLength = ncols * gridRes;
        totalWidth  = nrows * gridRes;
        totalSpan   = max(Zterrain(:), [], 'omitnan') - min(Zterrain(:), [], 'omitnan');
        
        title(sprintf('Fluid Road Inspector (Length: %.1fm | Width: %.1fm | Z-Span: %.3fm | 15x Z-Stretch)', ...
            totalLength, totalWidth, totalSpan));
        xlabel('Road Length X [m]'); ylabel('Road Width Y [m]'); zlabel('Elevation Z [m]');
        view(-30, 40);
        
        zoom(figFluid, 'on'); rotate3d(figFluid, 'on'); pan(figFluid, 'on'); 
        fprintf('  -> Fluid inspector ready! Use the figure toolbar icons to Pan, Zoom, or Rotate freely.\n');
    end
end