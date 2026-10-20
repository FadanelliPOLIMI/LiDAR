function [Xq, Yq, Z_safe, riskMap] = interpolate_BayesianKrigingRisk(ptCloudGround, resolution, confidenceLevel)
% INTERPOLATE_BAYESIANKRIGINGRISK - Sfrutta i Processi Gaussiani (Bayesiani) 
% per calcolare la superficie stradale e una mappa di rischio basata sulla varianza.

if nargin < 3, confidenceLevel = 1.96; end % 95% di intervallo di confidenza (2 sigma)

xyz = double(ptCloudGround.Location);
x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

% Binning preliminare per efficienza GPR
binSize = 0.2;
xMin = min(x); yMin = min(y);
xBinIdx = floor((x - xMin) / binSize) + 1;
yBinIdx = floor((y - yMin) / binSize) + 1;
validMask = ~isnan(z);
[~, ~, ic] = unique([xBinIdx(validMask), yBinIdx(validMask)], 'rows');
xb = accumarray(ic, x(validMask), [], @mean);
yb = accumarray(ic, y(validMask), [], @mean);
zb = accumarray(ic, z(validMask), [], @mean);

xGrid = min(x):resolution:max(x);
yGrid = min(y):resolution:max(y);
[Xq, Yq] = meshgrid(xGrid, yGrid);

% 1. Addestramento del Modello Bayesiano (GPR)
gprMdl = fitrgp([xb, yb], zb, ...
    'BasisFunction', 'linear', ...
    'KernelFunction', 'exponential', ...
    'Standardize', true);

% 2. Predizione Bayesiana: Otteniamo Media (mu) e Deviazione Standard (sigma)
[mu, sigma] = predict(gprMdl, [Xq(:), Yq(:)]);

Z_mean = reshape(mu, size(Xq));
Z_std  = reshape(sigma, size(Xq));

% 3. Calcolo del "Bayesian VaR" stradale (Scenario Worst-Case a 95% di confidenza)
% Se vuoi testare la robustezza strutturale in ADAMS considerando l'incertezza:
Z_safe = Z_mean + (confidenceLevel * Z_std); 

% 4. Creazione dell'indice di rischio spaziale basato sull'incertezza pura
% Normalizziamo la deviazione standard come misura del rischio di dato mancante/rumoroso
riskMap = Z_std / max(Z_std(:)); 

fprintf('[Bayesian Risk] Superficie e mappa di incertezza posteriore calcolate.\n');
fprintf('  * Incertezza media sulla pista (sigma) : %.4f metri\n', mean(Z_std(:)));
fprintf('  * Incertezza massima (Aree a rischio)  : %.4f metri\n', max(Z_std(:)));
end