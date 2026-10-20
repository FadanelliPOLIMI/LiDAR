function [Xq, Yq, Z_bspline] = interpolate_BSplineSurface(ptCloudGround, gridResolution)
% INTERPOLATE_BSPLINESURFACE - Ricostruzione superficie liscia con binning adattivo e fallback protetto
if nargin < 2
    gridResolution = 0.1; % Risoluzione OpenCRG (10 cm)
end

% Gestione flessibile dell'input (supporta sia object pointCloud che matrice [N x 3])
if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
else
    xyz = double(ptCloudGround);
end

x = xyz(:, 1); y = xyz(:, 2); z = xyz(:, 3);
validMask = ~isnan(z);
xV = x(validMask); yV = y(validMask); zV = z(validMask);

% BINNING ADATTIVO ANTI-CRASH (Tetto massimo: 2500 punti per garantire la stabilità di tpaps)
binSize = 0.3; 
xMin = min(xV); yMin = min(yV);
xBinIdx = floor((xV - xMin) / binSize) + 1;
yBinIdx = floor((yV - yMin) / binSize) + 1;
[~, ~, ic] = unique([xBinIdx, yBinIdx], 'rows');

maxAllowedPoints = 2500;
while length(unique(ic)) > maxAllowedPoints && binSize < 3.0
    binSize = binSize + 0.15;
    xBinIdx = floor((xV - xMin) / binSize) + 1;
    yBinIdx = floor((yV - yMin) / binSize) + 1;
    [~, ~, ic] = unique([xBinIdx, yBinIdx], 'rows');
end

xBin = accumarray(ic, xV, [], @mean);
yBin = accumarray(ic, yV, [], @mean);
zBin = accumarray(ic, zV, [], @mean);

fprintf('  -> [B-Spline RAM-Safe] Utilizzati %d punti binnati (Bin size: %.2fm).\n', length(xBin), binSize);

% Griglia di output regolare 2D
xGrid = min(x):gridResolution:max(x);
yGrid = min(y):gridResolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);

try
    % Tentativo principale con tpaps: smoothing a 0.96 per convergenza stabile
    surfaceModel = tpaps([xBin'; yBin'], zBin', 0.96);
    Z_bspline = fnval(surfaceModel, {xGrid, yGrid});
    Z_bspline = Z_bspline';
    fprintf('[B-Spline Surface] Generata con successo via tpaps.\n');
catch
    % Fallback robusto in caso di mancata convergenza
    fprintf('  -> Avviso: tpaps non ha converso. Attivo fallback geometrico robusto...\n');
    F = scatteredInterpolant(xBin, yBin, zBin, 'natural', 'linear');
    Z_raw = F(Xq, Yq);

    % Riempimento di eventuali buchi con nearest
    if any(isnan(Z_raw(:)))
        F_nearest = scatteredInterpolant(xBin, yBin, zBin, 'nearest', 'nearest');
        nanMask = isnan(Z_raw);
        Z_raw(nanMask) = F_nearest(Xq(nanMask), Yq(nanMask));
    end

    % Filtro di smoothing 2D per replicare la continuità C2 e rimuovere gli spikes
    Z_bspline = imgaussfilt(Z_raw, 1.2);
    fprintf('[B-Spline Surface] Generata con successo via fallback protetto.\n');
end
end