function [ZGrid, XGrid, YGrid, gridStruct] = interpolate_Spatial_Optimal(ptCloudGround, resolution)
% INTERPOLATE_SPATIAL_OPTIMAL - Interpolazione geostatistica dei punti di terra
% Converte la nuvola di punti irregolare in una griglia regolare OpenCRG-Ready per MSC ADAMS.
if nargin < 2
    resolution = 0.1; % Risoluzione standard di 10 cm per superfici stradali ADAMS
end

% Gestione flessibile dell'input (supporta sia object pointCloud che matrice [N x 3])
if isa(ptCloudGround, 'pointCloud')
    xyz = double(ptCloudGround.Location);
    numPoints = ptCloudGround.Count;
else
    xyz = double(ptCloudGround);
    numPoints = size(xyz, 1);
end

fprintf('  -> [Geostatistical Interpolation] Elaborazione punti di terra (%d punti)...\n', numPoints);

x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);

% 1. Definizione dei limiti spaziali della griglia stradale
xMin = min(x); xMax = max(x);
yMin = min(y); yMax = max(y);
fprintf('  -> Estensione tracciato: X [%.2f : %.2f] m, Y [%.2f : %.2f] m (Risoluzione: %.2f m)...\n', ...
    xMin, xMax, yMin, yMax, resolution);

[XGrid, YGrid] = meshgrid(xMin : resolution : xMax, yMin : resolution : yMax);

% 2. Interpolatore Geostatistico (Natural Neighbor)
fprintf('  -> Costruzione interpolatore spaziale (scatteredInterpolant)...\n');
F = scatteredInterpolant(x, y, z, 'natural', 'linear');

% 3. Valutazione delle quote sulla griglia regolare
ZGrid = F(XGrid, YGrid);

% 4. Gestione robusta dei bordi (Fallback Nearest Neighbor)
nanMask = isnan(ZGrid);
if any(nanMask(:))
    fprintf('  -> [Nota] Rilevati bordi non coperti: applicazione fallback Nearest Neighbor...\n');
    F_nearest = scatteredInterpolant(x, y, z, 'nearest', 'none');
    ZGrid(nanMask) = F_nearest(XGrid(nanMask), YGrid(nanMask));
end

% 5. Impacchettamento della struttura dati per MSC ADAMS / OpenCRG
gridStruct.X = XGrid;
gridStruct.Y = YGrid;
gridStruct.Z = ZGrid;
gridStruct.Resolution = resolution;
gridStruct.Dimensions = size(XGrid);
gridStruct.MinElevation = min(ZGrid(:));
gridStruct.MaxElevation = max(ZGrid(:));

fprintf('[Interpolation] Griglia regolare completata con successo [%d x %d nodi].\n', ...
    gridStruct.Dimensions(1), gridStruct.Dimensions(2));

% -----------------------------------------------------------------
% VISUALIZZAZIONE 3D DELLA SUPERFICIE STRADALE REGOLARE (Mesh / Surf)
% -----------------------------------------------------------------
figure('Name', 'OpenCRG Road Surface - Superficie Interpolata', 'Color', 'w');
surf(XGrid, YGrid, ZGrid, 'EdgeColor', 'none');
title(sprintf('Superficie Stradale Regolare OpenCRG-Ready | Risoluzione: %.1f cm', resolution * 100), ...
    'FontSize', 11, 'FontWeight', 'bold');
xlabel('X (m) [Longitudinale]', 'FontSize', 10);
ylabel('Y (m) [Trasversale]', 'FontSize', 10);
zlabel('Z (m) [Quota Altimetrica]', 'FontSize', 10);
colormap jet;
cb = colorbar;
cb.Label.String = 'Quota Z (m)';
view(3);
grid on;
axis equal;
shading interp; 
light; lighting gouraud;
end