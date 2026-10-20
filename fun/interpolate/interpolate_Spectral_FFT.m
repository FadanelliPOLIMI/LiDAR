function [Xq, Yq, Z_spectral] = interpolate_Spectral_FFT(ptCloudGround, gridResolution)
% INTERPOLATE_SPECTRAL_FFT - Filtraggio spettrale 2D (FFT) per la rimozione del rumore LiDAR
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

% 1. CREAZIONE GRIGLIA REGOLARE PRELIMINARE
xGrid = min(xV):gridResolution:max(xV);
yGrid = min(yV):gridResolution:max(yV);
[Xq, Yq] = meshgrid(xGrid, yGrid);

% 2. RASTERIZZAZIONE INIZIALE (Definizione di Z_mesh)
F = scatteredInterpolant(xV, yV, zV, 'natural', 'linear');
Z_mesh = F(Xq, Yq);

% Riempimento di sicurezza per eventuali buchi ai bordi
if any(isnan(Z_mesh(:)))
    F_nearest = scatteredInterpolant(xV, yV, zV, 'nearest', 'nearest');
    nanMask = isnan(Z_mesh);
    Z_mesh(nanMask) = F_nearest(Xq(nanMask), Yq(nanMask));
end

% 3. FILTRAGGIO SPETTRALE 2D (FFT Passa-Basso)
% Utile per eliminare le micro-rugosità e il rumore bianco tipico dei sensori LiDAR
F_fft = fft2(Z_mesh);

[nrows, ncols] = size(Z_mesh);
[u, v] = meshgrid(linspace(-1, 1, ncols), linspace(-1, 1, nrows));
D = sqrt(u.^2 + v.^2);

% Frequenza di taglio (cutoff) per attenuare le alte frequenze spurie
cutoff = 0.35; 
H = exp(-(D.^2) / (2 * cutoff^2)); 

% Applicazione del filtro nello spazio delle frequenze e inversione
F_filtered = F_fft .* fftshift(H);
Z_spectral = real(ifft2(F_filtered));

fprintf('[Spectral FFT] Superficie filtrata nel dominio spettrale con successo.\n')
end