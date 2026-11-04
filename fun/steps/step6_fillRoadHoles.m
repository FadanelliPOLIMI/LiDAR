function [ptCloudMerged, ptCloudSynthetic] = step6_fillRoadHoles(ptCloudInput, config)
    % STEP6_FILLROADHOLES - Full Bounding Box Domain Inpainting & Texture Matching
    % Fills ALL empty grid cells within [Xmin, Xmax] x [Ymin, Ymax] using 
    % Gradient-Aware Local Surface Plane Projection and Texture Synthesis.

    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    if ~isfield(config, 'voxelSize'), config.voxelSize = single(0.05); end
    if ~isfield(config, 'mode'), config.mode = 'laptop'; end

    ptCloudMerged = ptCloudInput;
    ptCloudSynthetic = pointCloud(zeros(0, 3, 'single'));
    
    if ptCloudInput.Count == 0, return; end

    fprintf('  -> [BOUNDING BOX INPAINTING] Detecting grid voids across full spatial domain...\n');
    tic;

    % --- 0. EXTRACT METADATA ---
    hasColor = isprop(ptCloudInput, 'Color') && ~isempty(ptCloudInput.Color);
    hasIntensity = isprop(ptCloudInput, 'Intensity') && ~isempty(ptCloudInput.Intensity);

    locs = single(ptCloudInput.Location);
    gridRes = single(config.voxelSize);

    minX = min(locs(:,1)); maxX = max(locs(:,1));
    minY = min(locs(:,2)); maxY = max(locs(:,2));

    xEdges = minX:gridRes:maxX;
    yEdges = minY:gridRes:maxY;

    numX = length(xEdges) - 1;
    numY = length(yEdges) - 1;

    if numX < 2 || numY < 2
        fprintf('  -> Point cloud spatial extent too small for grid rasterization.\n');
        return;
    end

    % --- 1. FAST VECTORIZED GRID RASTERIZATION ---
    [~, ~, ~, binX, binY] = histcounts2(locs(:,1), locs(:,2), xEdges, yEdges);

    validBins = (binX > 0 & binY > 0 & binX <= numX & binY <= numY);
    subX = binX(validBins);
    subY = binY(validBins);
    subZ = locs(validBins, 3);

    linIdx = sub2ind([numX, numY], subX, subY);

    % Fixed: Cast NaN fill value to single(NaN) to avoid accumarray class error
    ZVector = accumarray(linIdx, subZ, [numX * numY, 1], @min, single(NaN));
    ZGrid = reshape(ZVector, [numX, numY]);

    occupancyGrid = ~isnan(ZGrid);

    % --- 2. TARGET ALL EMPTY VOXELS UP TO (Xmin, Xmax, Ymin, Ymax) ---
    targetVoidGrid = ~occupancyGrid;
    [voidR, voidC] = find(targetVoidGrid);

    if isempty(voidR)
        fprintf('  -> Domain is 100%% dense. No empty cells detected.\n');
        return;
    end

    candidateX = single(xEdges(voidR)' + gridRes/2);
    candidateY = single(yEdges(voidC)' + gridRes/2);
    candidateX = candidateX(:);
    candidateY = candidateY(:);

    % --- 3. GRADIENT-AWARE LOCAL SURFACE PLANAR INPAINTING ---
    kdtreeReal = KDTreeSearcher(single(locs(:, 1:2)));
    K_neighbors = min(15, ptCloudInput.Count);

    [nnIdx, dists] = knnsearch(kdtreeReal, [candidateX, candidateY], 'K', K_neighbors);

    numCandidates = length(candidateX);
    synthZ = zeros(numCandidates, 1, 'single');
    localRoughness = zeros(numCandidates, 1, 'single');
    
    xCol = locs(:, 1); 
    yCol = locs(:, 2); 
    zCol = locs(:, 3);

    for i = 1:numCandidates
        nIdx = nnIdx(i, :);
        nDists = dists(i, :);

        nX = xCol(nIdx);
        nY = yCol(nIdx);
        nZ = zCol(nIdx);

        % Inverse Distance Weighting (IDW) baseline
        weights = 1.0 ./ (single(nDists).^2 + 1e-4);
        weights = weights / sum(weights);
        zIDW = sum(weights(:) .* nZ(:));

        % Local 1st-Order Plane Fit: Z = Z0 + a*dX + b*dY
        if length(nZ) >= 6
            dx = double(nX - candidateX(i));
            dy = double(nY - candidateY(i));
            dz = double(nZ - zIDW);

            A = [dx, dy];
            coeff = A \ dz;
            
            zPlane = zIDW - single(coeff(1)*0 + coeff(2)*0); 
            
            % Safety clamp to keep outer boundary points bounded (+/- 8 cm)
            planeDev = max(-0.08, min(0.08, zPlane - zIDW));
            synthZ(i) = zIDW + planeDev;
        else
            synthZ(i) = zIDW;
        end

        if length(nZ) > 1
            localRoughness(i) = std(nZ);
        end
    end

    % --- 4. MICRO-TEXTURE & METADATA INHERITANCE ---
    localRoughness = single(localRoughness(:));
    localRoughness = min(localRoughness, single(0.012)); % Cap micro-noise to 1.2 cm

    rng(42); 
    microNoise = randn(numCandidates, 1, 'single') .* localRoughness;
    synthZFinal = synthZ + microNoise;

    primaryNN = nnIdx(:, 1);

    if hasColor
        synthColor = ptCloudInput.Color(primaryNN, :);
        ptCloudSynthetic = pointCloud([candidateX, candidateY, synthZFinal], 'Color', synthColor);
    else
        ptCloudSynthetic = pointCloud([candidateX, candidateY, synthZFinal]);
    end

    if hasIntensity
        ptCloudSynthetic.Intensity = ptCloudInput.Intensity(primaryNN);
    end

    % --- 5. NON-DESTRUCTIVE MERGE ---
    ptCloudMerged = pcmerge(ptCloudInput, ptCloudSynthetic, config.voxelSize);

    elapsedTime = toc;
    fprintf('  -> Successfully inpainted full bounding box! Added %d synthetic points in %.2f sec.\n', ...
        ptCloudSynthetic.Count, elapsedTime);

    % --- 6. VISUALIZE ONLY THE SYNTHETIC BOUNDING-BOX INPAINTING POINTS ---
    hasDisplay = ~isempty(java.awt.GraphicsEnvironment.isHeadless) && ...
                 ~java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment().isHeadless();

    if isfield(config, 'visible') && config.visible && ~strcmpi(config.mode, 'edge') && hasDisplay
        figure('Name', 'Synthetic Full-Domain Inpainting Voids Only', 'Color', 'w');
        if hasColor
            pcshow(ptCloudSynthetic, 'MarkerSize', 45);
        else
            pcshow(ptCloudSynthetic.Location, synthZFinal, 'MarkerSize', 45);
            colormap(gca, 'jet');
        end
        title(sprintf('Inpainted Full Bounding Box Domain Only (%d synthetic points)', ptCloudSynthetic.Count));
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        grid on;
        drawnow;
    end
end