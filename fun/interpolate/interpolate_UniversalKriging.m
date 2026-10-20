function [Xq, Yq, Z_uk, Z_var, krrigStruct] = interpolate_UniversalKriging(ptCloudGround, gridResolution)
% INTERPOLATE_UNIVERSALKRIGING - Kriging Universale con binning adattivo anti-blocco (Max 2500 punti)
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

% --- BINNING ADATTIVO ANTI-BLOCCO (Tetto massimo: 2500 punti) ---
binSize = 0.4; 
xMin = min(xV); yMin = min(yV);
xBinIdx = floor((xV - xMin) / binSize) + 1;
yBinIdx = floor((yV - yMin) / binSize) + 1;
[~, ~, ic] = unique([xBinIdx, yBinIdx], 'rows');

maxAllowedPoints = 2500;
while length(unique(ic)) > maxAllowedPoints && binSize < 3.0
    binSize = binSize + 0.2;
    xBinIdx = floor((xV - xMin) / binSize) + 1;
    yBinIdx = floor((yV - yMin) / binSize) + 1;
    [~, ~, ic] = unique([xBinIdx, yBinIdx], 'rows');
end

xb = accumarray(ic, xV, [], @mean);
yb = accumarray(ic, yV, [], @mean);
zb = accumarray(ic, zV, [], @mean);

fprintf('  -> [Kriging RAM-Safe] Utilizzati %d punti binnati geostatistici (Bin size: %.2fm).\n', length(xb), binSize);

% Griglia di output regolare 2D
xGrid = min(x):gridResolution:max(x);
yGrid = min(y):gridResolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);

fprintf('  -> Addestramento modello Kriging Universale (Gaussian Process Regression)...\n');

% CORRETTO: Adesso fitrgp lavora sui dati binnati (xb, yb, zb) e non su 648k punti!
try
    gprMdl = fitrgp([xb, yb], zb, ...
        'BasisFunction', 'linear', ...
        'KernelFunction', 'exponential', ...
        'Standardize', true, ...
        'Verbose', 0);

    fprintf('  -> Predizione della quota e stima dell''incertezza sulla griglia 2D...\n');
    [Z_pred, Z_sd] = predict(gprMdl, [Xq(:), Yq(:)]);

    Z_uk  = reshape(Z_pred, size(Xq));
    Z_var = reshape(Z_sd.^2, size(Xq));

    krrigStruct.model = gprMdl;
    fprintf('[Universal Kriging] Superficie geostatistica generata con successo!\n');

catch ME
    warning('Kriging fallito (%s). Attivo fallback su Natural Neighbor...', ME.message);
    F = scatteredInterpolant(xb, yb, zb, 'natural', 'linear');
    Z_uk = F(Xq, Yq);
    Z_var = zeros(size(Xq));
    krrigStruct.model = [];
end
end