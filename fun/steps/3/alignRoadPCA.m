function [ptCloudRoadAligned, geoTransform, xMin, yMin, xMax, yMax, pcaFailed] = alignRoadPCA(ptCloudRoad, config)
    % ALIGNROADPCA - Geometric Alignment & Geo-Anchoring via 2D PCA
    % Preserves both Color (RGB) and Intensity completely without spatial distortion.
    % Conditionally executes only if config.laptop is true.
    
    % Default return values if laptop config is disabled or skipped
    geoTransform = struct();
    ptCloudRoadAligned = ptCloudRoad;
    pcaFailed = false; % Default to false

    X_global = ptCloudRoad.Location(:,1);
    Y_global = ptCloudRoad.Location(:,2);

    xMin = min(X_global); xMax = max(X_global);
    yMin = min(Y_global); yMax = max(Y_global);

    % Check if laptop configuration flag is set and true
    if nargin < 2 || ~isfield(config, 'mode') || ~strcmpi(config.mode, 'laptop')
        fprintf('  -> [Config] config.mode is not set to "laptop". Skipping PCA alignment.\n');
        return;
    end
    
    Z_orig = double(ptCloudRoad.Location(:,3));

    if isempty(X_global)
        error('PipelineError:EmptyInput', 'Input road point cloud is empty.');
    end
    
    % Safe attribute preservation
    hasColor = isprop(ptCloudRoad, 'Color') && ~isempty(ptCloudRoad.Color);
    hasIntensity = isprop(ptCloudRoad, 'Intensity') && ~isempty(ptCloudRoad.Intensity);
    colors_orig = []; intensity_orig = [];
    if hasColor, colors_orig = ptCloudRoad.Color; end
    if hasIntensity, intensity_orig = double(ptCloudRoad.Intensity); end
    
    % --- 2. PCA Alignment (Rigid 2D Rotation) ---
    fprintf('  -> Computing principal orientation and PCA rotation...\n');
    centroid = mean([X_global, Y_global], 1);
    X_cent = X_global - centroid(1);
    Y_cent = Y_global - centroid(2);
    
    [coeff, ~, latent] = pca([X_cent, Y_cent]);
   
    
    % CHECK FOR PCA FAILURE / ISOTROPY (Roundabouts, square tiles, or degenerate clouds)
    % If eigenvalue ratio is close to 1.0, there is no dominant road axis.
    if length(latent) < 2 || (latent(1) / max(latent(2), 1e-6)) < 1.2
        warning('PipelineWarning:PCAIsotropic', 'Point cloud lacks dominant linear direction (Eigenvalue ratio < 1.2). Bypassing PCA rotation.');
        R = eye(2);
        theta = 0;
        X_rot = X_cent;
        Y_rot = Y_cent;
        pcaFailed = true;
    else
        v_road = coeff(:,1);
        angleRad = atan2(v_road(2), v_road(1));
        theta = -angleRad; 

        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        rotatedCoords = [X_cent, Y_cent] * R';
        X_rot = rotatedCoords(:,1);
        Y_rot = rotatedCoords(:,2);
    end

    % --- 3. Local Translation ---
    xOffset = min(X_rot);
    yOffset = min(Y_rot);
    X_local = X_rot - xOffset;
    Y_local = Y_rot - yOffset;

    xMin = min(X_local); xMax = max(X_local);
    yMin = min(Y_local); yMax = max(Y_local);

    % --- 4. Point Cloud Reconstruction with Full Attributes ---
    coords_local = single([X_local, Y_local, Z_orig]);
    if hasColor && hasIntensity
        ptCloudRoadAligned = pointCloud(coords_local, 'Color', colors_orig, 'Intensity', uint16(intensity_orig));
    elseif hasColor
        ptCloudRoadAligned = pointCloud(coords_local, 'Color', colors_orig);
    elseif hasIntensity
        ptCloudRoadAligned = pointCloud(coords_local, 'Intensity', uint16(intensity_orig));
    else
        ptCloudRoadAligned = pointCloud(coords_local);
    end

    % --- CORRECTED NORMAL ROTATION ---
    if isprop(ptCloudRoad, 'Normal') && ~isempty(ptCloudRoad.Normal)
        normals_orig = single(ptCloudRoad.Normal);
        normals_2d_rot = normals_orig(:, 1:2) * single(R'); 
        ptCloudRoadAligned.Normal = [normals_2d_rot(:,1), normals_2d_rot(:,2), normals_orig(:,3)];
        fprintf('  -> Surface normal vectors successfully rotated to match local PCA space.\n');
    end

    % --- 5. Global Geo-Anchoring Storage (Clean struct, no anonymous functions) ---
    geoTransform.centroid = centroid;
    geoTransform.theta    = theta;
    geoTransform.R        = R;
    geoTransform.xOffset  = xOffset;
    geoTransform.yOffset  = yOffset;

    fprintf('  -> Road successfully straightened, localized, and geo-anchored.\n');
    fprintf('--------------------------------------------\n');
end

