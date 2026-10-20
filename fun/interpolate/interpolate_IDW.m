function [Xq, Yq, Z_idw] = interpolate_IDW(ptCloudGround, gridResolution, pPower, kNeighbors)
% INTERPOLATE_IDW - Inverse Distance Weighting ottimizzato con KD-Tree e vettorizzazione spinta
if nargin < 2, gridResolution = 0.1; end
if nargin < 3, pPower = 2.0; end
if nargin < 4, kNeighbors = 16; end

if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
else
    xyz = double(ptCloudGround);
end
x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

% 1. Definizione della griglia regolare 2D
xGrid = min(x):gridResolution:max(x);
yGrid = min(y):gridResolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);
xq_flat = Xq(:);
yq_flat = Yq(:);
numNodes = length(xq_flat);

fprintf('[IDW Ottimizzato] Costruzione KD-Tree per %d punti LiDAR...\n', length(x));
mdl = KDTreeSearcher([x, y]);

fprintf('[IDW Ottimizzato] Ricerca spaziale (K=%d) e calcolo vettorizzato per %d nodi...\n', ...
    kNeighbors, numNodes);

% Ricerca dei K vicini più vicini per tutti i nodi della griglia
[idx, dist] = knnsearch(mdl, [xq_flat, yq_flat], 'K', kNeighbors);

% Gestione divisioni per zero (punti esattamente coincidenti)
exactMatch = (dist == 0);
dist(exactMatch) = 1e-10; % Valore infinitesimale sicuro

% Calcolo dei pesi IDW w = 1 / (d^p)
weights = 1 ./ (dist .^ pPower);

% Estrazione delle quote Z dei vicini [numNodes x kNeighbors]
Z_neighbors = z(idx); 

% Media pesata IDW vettorizzata standard
sumWeights = sum(weights, 2);
sumWeightedZ = sum(weights .* Z_neighbors, 2);
z_flat = sumWeightedZ ./ sumWeights;

% OTTIMIZZAZIONE: Gestione punti coincidenti senza cicli inutili su 600k nodi
[rowExact, ~] = find(exactMatch);
if ~isempty(rowExact)
    uniqueExactRows = unique(rowExact);
    for r = uniqueExactRows'
        matchCols = exactMatch(r, :);
        matchIdx = idx(r, matchCols);
        z_flat(r) = mean(z(matchIdx));
    end
end

% 2. Rimappatura sulla griglia 2D regolare
Z_idw = reshape(z_flat, size(Xq));

% 3. Controllo di sicurezza sui bordi (Taglio estrapolazioni selvagge)
maxValidDist = gridResolution * 3.0;
minDist = dist(:, 1);
Z_idw(minDist > maxValidDist) = NaN;

fprintf('[IDW Ottimizzato] Griglia %dx%d generata con successo.\n', ...
    size(Z_idw, 1), size(Z_idw, 2));
end