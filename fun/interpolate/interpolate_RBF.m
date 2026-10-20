function [Xq, Yq, Z_rbf] = interpolate_RBF(ptCloudGround, gridResolution)
% INTERPOLATE_RBF - Ricostruzione RBF con binning adattivo e chunking blindato.
% Mantiene la memoria allocata costantemente al di sotto dei limiti di RAM.

if nargin < 2
    gridResolution = 0.1; % Risoluzione standard OpenCRG (10 cm)
end

if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
else
    xyz = double(ptCloudGround);
end

x = xyz(:, 1); y = xyz(:, 2); z = xyz(:, 3);
validMask = ~isnan(z);
xV = x(validMask); yV = y(validMask); zV = z(validMask);

% 1. BINNING ADATTIVO ANTI-CRASH (Tetto massimo di sicurezza: 2500 punti)
% Aumenta dinamicamente la dimensione del bin finché i nodi binnati non rientrano nel budget RAM.
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

xb = accumarray(ic, xV, [], @mean);
yb = accumarray(ic, yV, [], @mean);
zb = accumarray(ic, zV, [], @mean);

fprintf('  -> [RBF RAM-Safe] Utilizzati %d punti binnati (Bin size: %.2fm).\n', length(xb), binSize);

% 2. DEFINIZIONE GRIGLIA DI OUTPUT 2D
xGrid = min(x):gridResolution:max(x);
yGrid = min(y):gridResolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);

numPoints = length(xb);
epsilon = 1.0 / (binSize * 2);

% 3. RISOLUZIONE SISTEMA LINEARE RBF (Kernel Gaussiano con Regolarizzazione)
DX = bsxfun(@minus, xb, xb');
DY = bsxfun(@minus, yb, yb');
R2 = DX.^2 + DY.^2;
Phi = exp(-(epsilon^2) * R2);

lambda = 1e-4; % Smorzamento Tikhonov contro matrici mal condizionate
weights = (Phi + lambda * eye(numPoints)) \ zb;

% 4. VALUTAZIONE A PICCOLI CHUNK (Chunk Size = 5000 Nodi)
xq_flat = Xq(:);
yq_flat = Yq(:);
numNodes = length(xq_flat);
Z_flat = zeros(numNodes, 1);

chunkSize = 5000; % Dimensione blocco ultra-sicura per non saturare la RAM
numChunks = ceil(numNodes / chunkSize);

for c = 1:numChunks
    idxStart = (c - 1) * chunkSize + 1;
    idxEnd = min(c * chunkSize, numNodes);

    % Calcolo distanze euclidee solo per il blocco corrente
    Dist2_chunk = pdist2([xq_flat(idxStart:idxEnd), yq_flat(idxStart:idxEnd)], [xb, yb], 'euclidean').^2;
    Phi_chunk = exp(-(epsilon^2) * Dist2_chunk);

    Z_flat(idxStart:idxEnd) = Phi_chunk * weights;
end

Z_rbf = reshape(Z_flat, size(Xq));
fprintf('[Radial Basis Functions] Superficie RBF calcolata in sicurezza!\n');
end