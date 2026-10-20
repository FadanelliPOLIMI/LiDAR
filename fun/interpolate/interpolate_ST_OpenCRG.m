function [Sq, Tq, Z_st, centerlineInfo] = interpolate_ST_OpenCRG(ptCloudGround, gridResolution)
% INTERPOLATE_ST_OPENCRG - Mappatura nativa in coordinate di stazione (s) e offset (t)
% Estrae l'asse longitudinale con PCA e costruisce la griglia nel dominio s-t di OpenCRG.
if nargin < 2
    gridResolution = 0.1; 
end

% Gestione flessibile dell'input (supporta sia object pointCloud che matrice [N x 3])
if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
    numPoints = ptCloudGround.Count;
else
    xyz = double(ptCloudGround);
    numPoints = size(xyz, 1);
end

fprintf('  -> Estrazione coordinate per mappatura s-t OpenCRG (%d punti)...\n', numPoints);

x = xyz(:, 1); y = xyz(:, 2); z = xyz(:, 3);

% 1. INDIVIDUAZIONE DELL'ASSE LONGITUDINALE CON PCA (Principal Component Analysis)
% Trova la direzione principale di scorrimento della strada (asse s)
[coeff, score, ~] = pca([x, y]);

% Coordinate ruotate nel sistema della strada: s (longitudinale), t (trasversale)
s = score(:, 1); 
t = score(:, 2);

% Salvataggio informazioni asse per eventuale ricostruzione o export OpenCRG
centerlineInfo.coeff = coeff;
centerlineInfo.meanXY = mean([x, y], 1);

% 2. CREAZIONE GRIGLIA NEL DOMINIO s-t
sGrid = min(s):gridResolution:max(s);
tGrid = min(t):gridResolution:max(t);
[Sq, Tq] = meshgrid(sGrid, tGrid);

fprintf('  -> Interpolazione spaziale nel dominio s-t (Natural Neighbor)...\n');

% L'interpolazione nel piano s-t rispetta perfettamente la geometria intrinseca
F = scatteredInterpolant(s, t, z, 'natural', 'nearest');
Z_st = F(Sq, Tq);

% Riempimento di sicurezza buchi ai bordi
if any(isnan(Z_st(:)))
    F_lin = scatteredInterpolant(s, t, z, 'linear', 'nearest');
    nanMask = isnan(Z_st);
    Z_st(nanMask) = F_lin(Sq(nanMask), Tq(nanMask));
end

fprintf('[OpenCRG s-t Domain] Griglia di stazione e offset generata con successo!\n');
end