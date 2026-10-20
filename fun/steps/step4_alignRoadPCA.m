function [X_orig, Y_orig, Z_orig, xMin, yMin, xMax, yMax, ptCloudRoadAligned, geoTransform] = step4_alignRoadPCA(ptCloudRoad)
% STEP4_ALIGNROADPCA - Allineamento geometrico & Geo-ancoraggio (PCA)
% Preserva integralmente sia il Color (RGB) che l'Intensity.

fprintf('\n============================================\n');
fprintf('ALLINEAMENTO GEOMETRICO & GEO-ANCORAGGIO (PCA)\n');
fprintf('============================================\n');

X_global = double(ptCloudRoad.Location(:,1));
Y_global = double(ptCloudRoad.Location(:,2));
Z_orig   = double(ptCloudRoad.Location(:,3));

if isempty(X_global)
    error('La nuvola di punti stradale in ingresso è vuota.');
end

% --- 1. Gestione sicura dei Colori e dell'Intensity in ingresso ---
hasColor = false;
hasIntensity = false;

try
    if isprop(ptCloudRoad, 'Color') && ~isempty(ptCloudRoad.Color)
        colors_orig = ptCloudRoad.Color;
        if size(colors_orig, 1) == length(X_global)
            hasColor = true;
        end
    end
catch
end

try
    if isprop(ptCloudRoad, 'Intensity') && ~isempty(ptCloudRoad.Intensity)
        intensity_orig = double(ptCloudRoad.Intensity);
        if length(intensity_orig) == length(X_global)
            hasIntensity = true;
        end
    end
catch
end

% --- 2. Allineamento PCA (Rotazione rigida) ---
fprintf('  -> Calcolo orientamento principale e rotazione PCA...\n');
centroid = mean([X_global, Y_global], 1);
X_cent = X_global - centroid(1);
Y_cent = Y_global - centroid(2);

[coeff, ~] = pca([X_cent, Y_cent]);
v_road = coeff(:,1);
angleRad = atan2(v_road(2), v_road(1));
theta = -angleRad; 

R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
rotatedCoords = [X_cent, Y_cent] * R';
X_rot = rotatedCoords(:,1);
Y_rot = rotatedCoords(:,2);

% --- 3. Traslazione locale ---
xOffset = min(X_rot);
yOffset = min(Y_rot);
X_orig = X_rot - xOffset;
Y_orig = Y_rot - yOffset;

xMin = min(X_orig); xMax = max(X_orig);
yMin = min(Y_orig); yMax = max(Y_orig);

% --- 4. Ricostruzione Point Cloud con attributi completi ---
if hasColor && hasIntensity
    ptCloudRoadAligned = pointCloud([X_orig, Y_orig, Z_orig], 'Color', colors_orig, 'Intensity', intensity_orig);
elseif hasColor
    ptCloudRoadAligned = pointCloud([X_orig, Y_orig, Z_orig], 'Color', colors_orig);
elseif hasIntensity
    ptCloudRoadAligned = pointCloud([X_orig, Y_orig, Z_orig], 'Intensity', intensity_orig);
else
    ptCloudRoadAligned = pointCloud([X_orig, Y_orig, Z_orig]);
end

% --- 5. Salvataggio parametri di geo-ancoraggio globale ---
geoTransform.centroid = centroid;
geoTransform.theta    = theta;
geoTransform.R        = R;
geoTransform.xOffset  = xOffset;
geoTransform.yOffset  = yOffset;
geoTransform.localToGlobal = @(x_l, y_l) ...
    (([x_l + xOffset, y_l + yOffset] * R) + centroid);

fprintf('  -> Strada raddrizzata, posizionata in locale e geo-ancorata con successo.\n');
fprintf('============================================\n');
end