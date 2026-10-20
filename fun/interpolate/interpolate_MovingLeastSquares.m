function [Xq, Yq, Z_mls] = interpolate_MovingLeastSquares(ptCloudGround, gridResolution, searchRadius)
% INTERPOLATE_MOVINGLEASTSQUARES - Versione ottimizzata e parallelizzabile con KD-Tree
if nargin < 2, gridResolution = 0.1; end
if nargin < 3, searchRadius = 0.6; end 

if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
else
    xyz = double(ptCloudGround);
end
x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

% 1. Definizione della griglia regolare
xGrid = min(x):gridResolution:max(x);
yGrid = min(y):gridResolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);
xq_flat = Xq(:);
yq_flat = Yq(:);
numNodes = length(xq_flat);
Z_flat = nan(numNodes, 1);

fprintf('  -> Costruzione KD-Tree e ricerca spaziale rapida (Radius: %.2fm)...\n', searchRadius);
mdl = KDTreeSearcher([x, y]);

% Ricerca simultanea dei vicini nel raggio per tutti i nodi della griglia
[idxCell, distCell] = rangesearch(mdl, [xq_flat, yq_flat], searchRadius);

fprintf('  -> Esecuzione regressione pesata Moving Least Squares su %d nodi (Parallela)...\n', numNodes);

% Sostituito con parfor per sfruttare tutti i core della CPU in sicurezza
parfor i = 1:numNodes
    idx = idxCell{i};
    if length(idx) < 4
        continue; 
    end

    d2 = distCell{i}.^2;
    xn = x(idx) - xq_flat(i); 
    yn = y(idx) - yq_flat(i); 
    zn = z(idx);

    % Ponderazione gaussiana
    w = exp(-d2 / ((searchRadius/2)^2)); 

    % Matrice di disegno per i minimi quadrati pesati
    A = [xn, yn, ones(size(xn))];

    % Matrice normale con regolarizzazione di Tikhonov (Risoluzione diretta senza diag pesante)
    Aw = A .* sqrt(w);
    znw = zn .* sqrt(w);
    M = (Aw' * Aw) + 1e-6 * eye(3);
    rhs = Aw' * znw;

    % Risoluzione stabile
    params = M \ rhs; % Più efficiente e robusto di pinv(M)*rhs
    Z_flat(i) = params(3); 
end

% 2. Rimappatura sulla griglia 2D
Z_mls = reshape(Z_flat, size(Xq));

% 3. Riempimento sicuro dei buchi rimasti ai bordi
validMask = ~isnan(Z_mls);
if any(~validMask(:)) && sum(validMask(:)) > 10
    F_fallback = scatteredInterpolant(Xq(validMask), Yq(validMask), Z_mls(validMask), 'linear', 'nearest');
    Z_mls(~validMask) = F_fallback(Xq(~validMask), Yq(~validMask));
end

fprintf('[Moving Least Squares] Superficie completata con successo ed elevata stabilità.\n');
end