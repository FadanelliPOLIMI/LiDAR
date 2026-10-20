%% LAS -> ASCII GRID + PRJ (CON DENOISING STATISTICO E PIPELINE OTTIMIZZATA)
% Conversione LAS in:
%   1) Filtraggio statistico preventivo del rumore (SOR)
%   2) ASCII Raster (.asc) con gestione Gap Filling
%   3) Spatial Reference File (.prj)
%   4) Run TerrainReport
clear;
clc;

%% =========================================================
% SCELTA FILE .LAS
% ==========================================================
lasFile = uigetfile('*.las', 'Seleziona il file LAS');
if isequal(lasFile, 0)
    error('Nessun file LAS selezionato.');
end
[~, baseName, ~] = fileparts(lasFile);
ascFile = [baseName '.asc'];
prjFile = [baseName '.prj'];

resPercent = 0.25;        % [%] del lato minore dell'AOI
targetPtsPerCell = 4;     % punti LAS medi desiderati per cella
cellSizeQuantum = 0.001;  % quantizzazione finale della cella [m]
nodata = -9999;

%% =========================================================
% 1. LETTURA DEL FILE LAS
% ==========================================================
fprintf('Lettura del file LAS...\n');
reader = lasFileReader(lasFile);
[ptCloudRaw, ~] = readPointCloud(reader);
fprintf('Punti iniziali nel LAS: %d\n', ptCloudRaw.Count);

%% =========================================================
% 2. LETTURA DEL CRS DAL LAS
% =========================================================
fprintf('Lettura del sistema di riferimento...\n');
try
    wktRecord = reader.readVLR(2112);
    wkt = wktRecord.Data;
    fprintf('CRS trovato nel LAS.\n');
catch
    warning(['Il LAS non contiene il VLR 2112 con il WKT del CRS. ' ...
        'Il file PRJ non verrà creato.']);
    wkt = '';
end

%% =========================================================
% 3. DENOISING STATISTICO PREVENTIVO (SOR - Statistical Outlier Removal)
% Pulisce rumore bianco, polvere e punti fluttuanti prima di qualsiasi elaborazione
% =========================================================
fprintf('Applicazione Denoising Statistico (SOR) sul file grezzo...\n');
kNeighbors = 15;   % Numero di vicini da analizzare per ogni punto
stdThresh  = 1.5;   % Soglia di deviazione standard per considerare un punto come rumore

[ptCloudClean, ~, outlierIdx] = pcdenoise(ptCloudRaw, ...
    'NumNeighbors', kNeighbors, 'Threshold', stdThresh);

fprintf('  -> Punti di rumore isolato rimossi: %d (%.1f%%)\n', ...
    sum(outlierIdx), (sum(outlierIdx) / ptCloudRaw.Count) * 100);
fprintf('  -> Punti puliti rimanenti: %d\n', ptCloudClean.Count);

%% =========================================================
% 4. ESTRAZIONE E PULIZIA INIZIALE (Nuvola Pulita Originale)
% =========================================================
X_raw = double(ptCloudClean.Location(:,1));
Y_raw = double(ptCloudClean.Location(:,2));
Z_raw = double(ptCloudClean.Location(:,3));

valid_raw = isfinite(X_raw) & isfinite(Y_raw) & isfinite(Z_raw);
X_raw = X_raw(valid_raw);
Y_raw = Y_raw(valid_raw);
Z_raw = Z_raw(valid_raw);

%% =========================================================
% 5. TECNICA DI OTTIMIZZAZIONE 1: VOXEL GRID DOWNSAMPLING PREVENTIVO
% =========================================================
tic;
voxelSize = 0.05; 
fprintf('Applicazione Voxel Grid Downsampling (Risoluzione: %.2f m)...\n', voxelSize);
ptCloud = pcdownsample(ptCloudClean, 'gridAverage', voxelSize);
fprintf('Punti dopo il downsampling: %d (Riduzione complessiva: %.1f%%)\n', ...
    ptCloud.Count, (1 - ptCloud.Count/ptCloudRaw.Count)*100);

X = double(ptCloud.Location(:,1));
Y = double(ptCloud.Location(:,2));
Z = double(ptCloud.Location(:,3));

valid = isfinite(X) & isfinite(Y) & isfinite(Z);
X = X(valid);
Y = Y(valid);
Z = Z(valid);

%% =========================================================
% 6. SELEZIONE GRAFICA DELLA PORZIONE DI NUVOLA (AOI)
% =========================================================
figure('Name', 'Selezione AOI Ottimizzata e Denoised');
scatter(X, Y, 1, Z, '.'); axis equal; axis tight;
xlabel('X [m]');
ylabel('Y [m]');
title('Seleziona l''area stradale --> INVIO');
colorbar; colormap(parula);

roi = drawrectangle();
pause;
pos = roi.Position;
xMin = pos(1);
yMin = pos(2);
xMax = xMin + pos(3);
yMax = yMin + pos(4);

% Mascheramento sulla nuvola ridotta
mask = X >= xMin & X <= xMax & Y >= yMin & Y <= yMax;
X = X(mask);
Y = Y(mask);
Z = Z(mask);

if isempty(Z)
    close;
    error('La porzione selezionata non contiene punti LAS validi.');
end

% Mascheramento analogo sulla nuvola ORIGINALE PULITA (per il confronto)
mask_raw = X_raw >= xMin & X_raw <= xMax & Y_raw >= yMin & Y_raw <= yMax;
X_orig = X_raw(mask_raw);
Y_orig = Y_raw(mask_raw);
Z_orig = Z_raw(mask_raw);

%% =========================================================
% 7. RISOLUZIONE DEM AUTOMATICA DALL'AOI + DENSITA' LAS
% ==========================================================
aoiWidth  = xMax - xMin;
aoiHeight = yMax - yMin;
aoiArea   = aoiWidth * aoiHeight;
if aoiWidth <= 0 || aoiHeight <= 0 || aoiArea <= 0
    close;
    error('Dimensioni non valide del rettangolo selezionato.');
end

numPtsAOI = numel(Z);
pointDensity = numPtsAOI / aoiArea;        
if ~isfinite(pointDensity) || pointDensity <= 0
    close;
    error('Impossibile stimare la densita'' dei punti LAS nell''AOI.');
end
lasSpacing = 1 / sqrt(pointDensity);       
cellSizeLAS = sqrt(targetPtsPerCell / pointDensity);
refSize = min(aoiWidth, aoiHeight);
cellSizePercent = refSize * (resPercent / 100);
cellSizeRaw = max(cellSizePercent, cellSizeLAS);
cellSize = ceil(cellSizeRaw / cellSizeQuantum) * cellSizeQuantum;
cellSize = max(cellSize, cellSizeQuantum);

fprintf('CELLSIZE DEM impostato: %.3f m\n', cellSize);
close;

%% =========================================================
% 8. LIMITI E DIMENSIONI DELLA GRIGLIA
% =========================================================
minX = floor(min(X)); maxX = ceil(max(X));
minY = floor(min(Y)); maxY = ceil(max(Y));
ncols = ceil((maxX - minX) / cellSize);
nrows = ceil((maxY - minY) / cellSize);

%% =========================================================
% 9. VALUTAZIONE ADATTIVA: ORIGINALE PULITA VS VOXEL
% =========================================================
fprintf('\n============================================\n');
fprintf('VALUTAZIONE AFFIDABILITA'' DOWNSAMPLING (VOXEL vs ORIGINAL)\n');
fprintf('============================================\n');

% Griglia con dati ORIGINALI PULITI
colIdxOrig = floor((X_orig - minX) / cellSize) + 1;
rowIdxOrig = floor((Y_orig - minY) / cellSize) + 1;
validOrig = colIdxOrig >= 1 & colIdxOrig <= ncols & rowIdxOrig >= 1 & rowIdx <= nrows;
gridOriginal = accumarray([rowIdxOrig(validOrig) colIdxOrig(validOrig)], Z_orig(validOrig), [nrows ncols], @mean, NaN);

% Griglia con dati DOWNROMPATI (Voxel)
colIdxDown = floor((X - minX) / cellSize) + 1;
rowIdxDown = floor((Y - minY) / cellSize) + 1;
validDown = colIdxDown >= 1 & colIdxDown <= ncols & rowIdxDown >= 1 & rowIdx <= nrows;
gridDown = accumarray([rowIdxDown(validDown) colIdxDown(validDown)], Z(validDown), [nrows ncols], @mean, NaN);

% Calcolo scarti
validCells = ~isnan(gridOriginal) & ~isnan(gridDown);
if any(validCells(:))
    deltaZ = abs(gridOriginal(validCells) - gridDown(validCells));
    meanBias = mean(deltaZ);
    maxError = max(deltaZ);
    rmseVal = sqrt(mean(deltaZ.^2));
    
    fprintf('  -> Errore medio (Bias)     : %.4f mm\n', meanBias * 1000);
    fprintf('  -> Errore massimo di quota : %.4f mm\n', maxError * 1000);
    fprintf('  -> RMSE geometrico         : %.4f mm\n', rmseVal * 1000);
else
    warning('Sovrapposizione insufficiente tra griglia originale e downsamplata.');
    rmseVal = 0;
end
fprintf('============================================\n');

%% =========================================================
% 10. ASSEGNAZIONE E CALCOLO DELLA GRIGLIA DEFINITIVA (ADATTIVO)
% =========================================================
rmseThreshold = 0.005; % Soglia di tolleranza 5 mm

if exist('rmseVal', 'var') && ~isempty(rmseVal) && rmseVal <= rmseThreshold
    fprintf('\n[LOGICA ADATTIVA] RMSE (%.2f mm) <= Soglia (%.1f mm).\n', rmseVal*1000, rmseThreshold*1000);
    fprintf('-> Utilizzo la nuvola ottimizzata con Voxel Grid (Massima efficienza per trasmissione/simulazione).\n');
    colIdx = colIdxDown(validDown);
    rowIdx = rowIdxDown(validDown);
    Z_final = Z(validDown);
else
    fprintf('\n[LOGICA ADATTIVA] Fallback automatico alla nuvola originale pulita.\n');
    colIdx = colIdxOrig(validOrig);
    rowIdx = rowIdxOrig(validOrig);
    Z_final = Z_orig(validOrig);
end

fprintf('\nCalcolo della griglia delle quote definitiva...\n');
grid = accumarray([rowIdx colIdx], Z_final, [nrows ncols], @mean, nodata);

numNoData = sum(grid(:) == nodata);
fprintf('Celle NODATA: %d\n', numNoData);
fprintf('Celle valide: %d\n', sum(grid(:) ~= nodata));

%% =========================================================
% 11. SCRITTURA DEL FILE ASC
% ==========================================================
fprintf('\nScrittura del file ASC...\n');
fid = fopen(ascFile, 'w');
if fid == -1, error('Impossibile creare il file %s.', ascFile); end

fprintf(fid, 'NCOLS %d\n', ncols);
fprintf(fid, 'NROWS %d\n', nrows);
fprintf(fid, 'XLLCORNER %.6f\n', minX);
fprintf(fid, 'YLLCORNER %.6f\n', minY);
fprintf(fid, 'CELLSIZE %.6f\n', cellSize);
fprintf(fid, 'NODATA_VALUE %.6f\n', nodata);

for r = nrows:-1:1
    fprintf(fid, '%g', grid(r,1));
    for c = 2:ncols
        fprintf(fid, ' %g', grid(r,c));
    end
    fprintf(fid, '\n');
end
fclose(fid);

%% =========================================================
% 12. SCRITTURA DEL FILE PRJ
% ==========================================================
if ~isempty(wkt)
    fprintf('Scrittura del file PRJ...\n');
    fid = fopen(prjFile, 'w');
    if fid == -1, error('Impossibile creare il file %s.', prjFile); end
    fprintf(fid, '%s', wkt);
    fclose(fid);
    fprintf('PRJ creato correttamente.\n');
else
    fprintf('PRJ non creato: CRS non presente nel LAS.\n');
end

elapsedTime = toc;
fprintf('\nTempo totale elaborazione: %.2f secondi.\n', elapsedTime);

%% =========================================================
% 13. VISUALIZZAZIONE DELLA GRIGLIA E CONFRONTO
% ==========================================================
gridPlot = grid;
gridPlot(gridPlot == nodata) = NaN;

figure('Name', 'Controllo Qualita Raster: Originale vs Voxel Denoised', 'Position', [100, 100, 1200, 500]);
subplot(1, 2, 1);
imagesc([minX maxX], [minY maxY], gridOriginal);
axis equal; axis tight; set(gca, 'YDir', 'normal'); colorbar;
title('Raster da Nuvola ORIGINALE PULITA');
xlabel('X [m]'); ylabel('Y [m]'); colormap(parula);

subplot(1, 2, 2);
imagesc([minX maxX], [minY maxY], gridPlot);
axis equal; axis tight; set(gca, 'YDir', 'normal'); colorbar;
title(sprintf('Raster Ottimizzato - RMSE: %.2f mm', rmseVal*1000));
xlabel('X [m]'); ylabel('Y [m]'); colormap(parula);

%% =========================================================
% 14. LETTURA DEL FILE ASC E PREPARAZIONE PER TERRAINREPORT
% ==========================================================
filename = ascFile;
fid = fopen(filename, 'r');
header = cell(6,1);
for i = 1:6, header{i} = fgetl(fid); end
fclose(fid);

CELLSIZE = sscanf(header{5}, 'CELLSIZE %f');
samplerate = 1 / CELLSIZE;

Zterrain = readmatrix(filename, 'FileType', 'text', 'NumHeaderLines', 6);
Zterrain = flipud(Zterrain);

%% =========================================================
% 15. RIELABORAZIONE DEI NODATA PER TERRAINREPORT (Gap Filling)
% ==========================================================
fprintf('\n============================================\n');
fprintf('INTERPOLAZIONE NODATA PER TERRAINREPORT\n');
fprintf('============================================\n');
maskNoData = (Zterrain == nodata);
numNoDataTerrain = sum(maskNoData(:));

if numNoDataTerrain > 0
    Zterrain(maskNoData) = NaN;
    [nRowsTerrain, nColsTerrain] = size(Zterrain);
    [Xgrid, Ygrid] = meshgrid(1:nColsTerrain, 1:nRowsTerrain);
    maskValid = isfinite(Zterrain);
    
    if sum(maskValid(:)) < 3
        error('Numero insufficiente di celle valide per eseguire l''interpolazione.');
    end
    
    F = scatteredInterpolant(Xgrid(maskValid), Ygrid(maskValid), Zterrain(maskValid), 'natural', 'nearest');
    Zterrain(maskNoData) = F(Xgrid(maskNoData), Ygrid(maskNoData));
end

%% =========================================================
% 16. CONFIGURAZIONE TRACK WIDTH E AVVIO TERRAINREPORT
% =========================================================
[nRowsTW, nColsTW] = size(Zterrain);
maxTrackCells = min(nRowsTW, nColsTW) - 1;
demStep = 1 / samplerate;
demExtentX = (nColsTW - 1) * demStep;
demExtentY = (nRowsTW - 1) * demStep;
defaultTrackCells = round(min(2.0, 0.25 * min(demExtentX, demExtentY)) * samplerate);
defaultTrackCells = max(1, min(defaultTrackCells, maxTrackCells));
defaultTrackWidth = defaultTrackCells * demStep;

trackWidth = defaultTrackWidth; 

GIS.mode = 'real';
GIS.units = 'm';
GIS.Zmatrix = Zterrain;
GIS.TrackWidth = trackWidth;
GIS.samplerate = samplerate;
GIS.CutOff = 10;
GIS.Fext = {};

risposta = questdlg('Vuoi continuare con TerrainReport?', 'Conferma', 'Sì', 'No', 'Sì');
if strcmp(risposta, 'Sì')
    figs = findall(groot,'Type','figure');
    for k = 1:numel(figs)
        try, figs(k).WindowState = 'minimized'; catch, end
    end
    TerrainReport_rectangular(GIS);
end





















%% =========================================================
% PARAMETRI DI TRASMISSIONE WIRELESS E PACCHETTI (Il tuo Limite)
% ==========================================================
enableWirelessLimit = true;     % Attiva il controllo di banda via etere
maxBandwidth_kBps   = 250;      % Banda massima disponibile stimata [KB/s]
vehicleSpeed_kmh    = 50;       % Velocità stimata del veicolo [km/h]
swathWidth          = 3.0;      % Larghezza della striscia stradale [m]

if enableWirelessLimit
    % Calcolo del budget di punti in base alla banda radio
    bytesPerPoint = 12; // X, Y, Z come float32
    maxPointsPerSec = (maxBandwidth_kBps * 1024) / bytesPerPoint;

    v = vehicleSpeed_kmh / 3.6; // m/s
    areaRate = v * swathWidth;  // m^2/s

    % Densità massima di punti trasmissibili per m^2
    rho_transmission = maxPointsPerSec / areaRate;

    % Spaziatura minima derivata dalla banda radio [m]
    voxelSizeFromBandwidth = 1 / sqrt(rho_transmission);

    fprintf('\n--- VINCOLO DI TRASMISSIONE VIA ETERE ---\n');
    fprintf('Banda massima stimata : %.1f KB/s\n', maxBandwidth_kBps);
    fprintf('Punti max per secondo : %d pt/s\n', round(maxPointsPerSec));
    fprintf('Spaziatura min pacchetti: %.3f m (Voxel consigliato)\n', voxelSizeFromBandwidth);

    % Forziamo il voxelSize in base alla banda radio (con un minimo di sicurezza di 2cm)
    voxelSize = max(0.02, voxelSizeFromBandwidth);
else
    voxelSize = 0.05; % Valore geometrico fisso standard
end