function [Xq, Yq, Z_tp] = interpolate_ThinPlateSpline(ptCloudGround, gridResolution)
% INTERPOLATE_THINPLATESPLINE - Genera superficie con TPS gestendo il limite di memoria
% Applica un binning adattivo per evitare che tpaps crei matrici N x N giganti.

if nargin < 2
    gridResolution = 0.1; 
end

if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
else
    xyz = double(ptCloudGround);
end

x = xyz(:, 1); y = xyz(:, 2); z = xyz(:, 3);
validMask = ~isnan(z);
xV = x(validMask); yV = y(validMask); zV = z(validMask);

% 1. BINNING ADATTIVO ANTI-CRASH (Tetto massimo di sicurezza per tpaps: 2500 punti)
binSize = 0.25; 
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

xBinCenters = accumarray(ic, xV, [], @mean);
yBinCenters = accumarray(ic, yV, [], @mean);
zBinMeans   = accumarray(ic, zV, [], @mean);

fprintf('  -> [TPS RAM-Safe] Utilizzati %d bin attivi (Bin size: %.2fm) per tpaps.\n', ...
    length(zBinMeans), binSize);

% 2. DEFINIZIONE GRIGLIA DI OUTPUT
xGrid = min(x):gridResolution:max(x);
yGrid = min(y):gridResolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);

% 3. CALCOLO THIN PLATE SPLINE (tpaps) SULLA SUPERFICIE CONTROLLATA
% Il parametro 0.95 applica uno smoothing leggero che evita oscillazioni spurie
surfaceModel = tpaps([xBinCenters'; yBinCenters'], zBinMeans', 0.95);

% 4. VALUTAZIONE VALORI SULLA GRIGLIA 2D
Z_tp = fnval(surfaceModel, {xGrid, yGrid});
Z_tp = Z_tp'; % Adattamento orientamento della matrice

fprintf('[Thin Plate Spline] Superficie C2 generata con successo.\n');
end