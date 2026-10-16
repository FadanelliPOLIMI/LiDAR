%% LAS -> ASCII GRID + PRJ
% Conversione LAS in:
%   1) ASCII Raster (.asc)
%   2) Spatial Reference File (.prj)
%   3) Run TerrainReport
%
% IMPORTANTE:
% Il file ASC mantiene i valori NODATA = -9999.
% Solo successivamente, quando l'ASC viene riletto per TerrainReport,
% i NODATA vengono interpolati.

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

% Parametri per la scelta AUTOMATICA della risoluzione DEM.
% La risoluzione finale viene calcolata DOPO la selezione rettangolare,
% combinando:
%   1) una quota percentuale del lato minore dell'AOI;
%   2) la densita' reale dei punti LAS contenuti nell'AOI.
%
% resPercent = 0.25 significa che il lato minore dell'AOI viene diviso
% idealmente in circa 400 celle (100/0.25 = 400), salvo il limite imposto
% dalla densita' LAS.
resPercent = 0.25;        % [%] del lato minore dell'AOI
targetPtsPerCell = 4;     % punti LAS medi desiderati per cella
cellSizeQuantum = 0.001;  % quantizzazione finale della cella [m]

nodata = -9999;


%% =========================================================
% 1. LETTURA DEL FILE LAS
% ==========================================================

fprintf('Lettura del file LAS...\n');

reader = lasFileReader(lasFile);

[ptCloud, ~] = readPointCloud(reader);


%% =========================================================
% 2. ESTRAZIONE DELLE COORDINATE
% ==========================================================

X = double(ptCloud.Location(:,1));
Y = double(ptCloud.Location(:,2));
Z = double(ptCloud.Location(:,3));


%% =========================================================
% 3. RIMOZIONE DEI PUNTI NON VALIDI
% ==========================================================

valid = isfinite(X) & isfinite(Y) & isfinite(Z);

X = X(valid);
Y = Y(valid);
Z = Z(valid);

fprintf('Numero di punti validi: %d\n', length(Z));


%% =========================================================
% 4. LETTURA DEL CRS DAL LAS
% ==========================================================

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
% 5. SELEZIONE GRAFICA DELLA PORZIONE DI NUVOLA
% ==========================================================

fprintf('\nApertura della finestra di selezione...\n');

figure;

scatter(X, Y, 1, Z, '.');

axis equal;
axis tight;

xlabel('X [m]');
ylabel('Y [m]');
title('Disegna un rettangolo --> INVIO per confermare');

colorbar;
colormap(parula);

% Disegna il rettangolo
roi = drawrectangle();

% Aspetta che l'utente prema INVIO
pause;

% Coordinate del rettangolo
pos = roi.Position;

xMin = pos(1);
yMin = pos(2);

xMax = xMin + pos(3);
yMax = yMin + pos(4);

% Selezione dei punti
mask = X >= xMin & X <= xMax & ...
       Y >= yMin & Y <= yMax;

X = X(mask);
Y = Y(mask);
Z = Z(mask);

% Controllo di sicurezza: il rettangolo deve contenere punti LAS.
if isempty(Z)
    close;
    error('La porzione selezionata non contiene punti LAS validi.');
end

%% =========================================================
% 5A. RISOLUZIONE DEM AUTOMATICA DALL'AOI + DENSITA' LAS
% ==========================================================
%
% Il LAS non possiede una risoluzione raster unica. Qui viene stimata una
% spaziatura caratteristica dalla densita' dei punti nell'area selezionata:
%
%   rho      = N / Area                  [punti/m^2]
%   hLAS     = 1/sqrt(rho)               [m]
%
% Inoltre imponiamo un numero medio target di punti per cella:
%
%   cellSizeLAS = sqrt(targetPtsPerCell/rho)
%
% In parallelo viene calcolata una risoluzione geometrica percentuale
% rispetto al lato minore del rettangolo selezionato. La risoluzione finale
% e' il valore PIU' CONSERVATIVO (il maggiore tra i due), in modo da non
% creare un raster inutilmente piu' fine di quanto la densita' LAS supporti.

aoiWidth  = xMax - xMin;
aoiHeight = yMax - yMin;
aoiArea   = aoiWidth * aoiHeight;

if aoiWidth <= 0 || aoiHeight <= 0 || aoiArea <= 0
    close;
    error('Dimensioni non valide del rettangolo selezionato.');
end

numPtsAOI = numel(Z);
pointDensity = numPtsAOI / aoiArea;        % [punti/m^2]

if ~isfinite(pointDensity) || pointDensity <= 0
    close;
    error('Impossibile stimare la densita'' dei punti LAS nell''AOI.');
end

lasSpacing = 1 / sqrt(pointDensity);       % [m], spaziatura equivalente
cellSizeLAS = sqrt(targetPtsPerCell / pointDensity);

refSize = min(aoiWidth, aoiHeight);
cellSizePercent = refSize * (resPercent / 100);

% Il max evita di chiedere al DEM una risoluzione piu' fine di quella
% ragionevolmente sostenuta dalla densita' della nuvola selezionata.
cellSizeRaw = max(cellSizePercent, cellSizeLAS);

% Quantizzazione al millimetro, arrotondando verso l'alto per restare
% conservativi rispetto al valore appena calcolato.
cellSize = ceil(cellSizeRaw / cellSizeQuantum) * cellSizeQuantum;
cellSize = max(cellSize, cellSizeQuantum);

fprintf('\n============================================\n');
fprintf('PORZIONE SELEZIONATA\n');
fprintf('============================================\n');

fprintf('X: %.3f -> %.3f m\n', xMin, xMax);
fprintf('Y: %.3f -> %.3f m\n', yMin, yMax);
fprintf('Dimensioni AOI: %.3f x %.3f m\n', aoiWidth, aoiHeight);
fprintf('Area AOI: %.3f m^2\n', aoiArea);
fprintf('Punti mantenuti: %d\n', numPtsAOI);
fprintf('Densita'' LAS: %.3f punti/m^2\n', pointDensity);
fprintf('Spaziatura LAS equivalente: %.4f m\n', lasSpacing);
fprintf('Cella da criterio percentuale (%.3f%%): %.4f m\n', ...
    resPercent, cellSizePercent);
fprintf('Cella da densita'' LAS (%g pt/cella): %.4f m\n', ...
    targetPtsPerCell, cellSizeLAS);
fprintf('CELLSIZE DEM automatico: %.3f m\n', cellSize);

fprintf('============================================\n');

close;


%% =========================================================
% 6. LIMITI DELLA NUVOLA
% ==========================================================

minX = floor(min(X));
maxX = ceil(max(X));

minY = floor(min(Y));
maxY = ceil(max(Y));


%% =========================================================
% 7. DIMENSIONE DELLA GRIGLIA
% ==========================================================

ncols = ceil((maxX - minX) / cellSize);
nrows = ceil((maxY - minY) / cellSize);

fprintf('\n');
fprintf('============================================\n');
fprintf('PARAMETRI GRIGLIA\n');
fprintf('============================================\n');

fprintf('Limiti X: %.3f -> %.3f\n', minX, maxX);
fprintf('Limiti Y: %.3f -> %.3f\n', minY, maxY);

fprintf('Numero colonne: %d\n', ncols);
fprintf('Numero righe:   %d\n', nrows);

fprintf('Risoluzione: %.3f m\n', cellSize);


%% =========================================================
% 8. ASSEGNAZIONE DEI PUNTI ALLA GRIGLIA
% ==========================================================

colIdx = floor((X - minX) / cellSize) + 1;
rowIdx = floor((Y - minY) / cellSize) + 1;


%% =========================================================
% 9. CONTROLLO DEGLI INDICI
% ==========================================================

valid = ...
    colIdx >= 1 & colIdx <= ncols & ...
    rowIdx >= 1 & rowIdx <= nrows;

colIdx = colIdx(valid);
rowIdx = rowIdx(valid);
Z = Z(valid);


%% =========================================================
% 10. CALCOLO DELLA QUOTA MEDIA
% ==========================================================
%
% ATTENZIONE:
% qui le celle prive di punti DEVONO rimanere -9999.
% Il file ASC viene quindi prodotto correttamente con i NODATA.

fprintf('\nCalcolo della griglia delle quote...\n');

grid = accumarray( ...
    [rowIdx colIdx], ...
    Z, ...
    [nrows ncols], ...
    @mean, ...
    nodata);


%% =========================================================
% 11. CONTROLLO NODATA
% ==========================================================

numNoData = sum(grid(:) == nodata);

fprintf('Celle NODATA: %d\n', numNoData);

fprintf('Celle valide: %d\n', ...
    sum(grid(:) ~= nodata));


%% =========================================================
% 12. SCRITTURA DEL FILE ASC
% ==========================================================

fprintf('\nScrittura del file ASC...\n');

fid = fopen(ascFile, 'w');

if fid == -1
    error('Impossibile creare il file %s.', ascFile);
end


% Header ESRI ASCII Raster

fprintf(fid, 'NCOLS %d\n', ncols);
fprintf(fid, 'NROWS %d\n', nrows);

fprintf(fid, 'XLLCORNER %.6f\n', minX);
fprintf(fid, 'YLLCORNER %.6f\n', minY);

fprintf(fid, 'CELLSIZE %.6f\n', cellSize);
fprintf(fid, 'NODATA_VALUE %.6f\n', nodata);


%% Scrittura delle quote
%
% ESRI ASCII Raster:
% prima riga = NORD
% ultima riga = SUD

for r = nrows:-1:1

    fprintf(fid, '%g', grid(r,1));

    for c = 2:ncols

        fprintf(fid, ' %g', grid(r,c));

    end

    fprintf(fid, '\n');

end

fclose(fid);


%% =========================================================
% 13. SCRITTURA DEL FILE PRJ
% ==========================================================

if ~isempty(wkt)

    fprintf('Scrittura del file PRJ...\n');

    fid = fopen(prjFile, 'w');

    if fid == -1
        error('Impossibile creare il file %s.', prjFile);
    end

    fprintf(fid, '%s', wkt);

    fclose(fid);

    fprintf('PRJ creato correttamente.\n');

else

    fprintf('PRJ non creato: CRS non presente nel LAS.\n');

end


%% =========================================================
% 14. INFORMAZIONI FINALI
% ==========================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('       CONVERSIONE COMPLETATA\n');
fprintf('============================================\n');

fprintf('Input LAS : %s\n', lasFile);
fprintf('Output ASC: %s\n', ascFile);

if ~isempty(wkt)
    fprintf('Output PRJ: %s\n', prjFile);
end

fprintf('\n');

fprintf('Numero punti : %d\n', length(Z));

fprintf('XLLCORNER   : %.2f m\n', minX);
fprintf('YLLCORNER   : %.2f m\n', minY);

fprintf('NCOLS       : %d\n', ncols);
fprintf('NROWS       : %d\n', nrows);

fprintf('CELLSIZE    : %.3f m\n', cellSize);

fprintf('Quota minima: %.3f m\n', min(Z));
fprintf('Quota massima: %.3f m\n', max(Z));

fprintf('NODATA      : %d celle\n', numNoData);

fprintf('============================================\n');


%% =========================================================
% INFORMAZIONI SISTEMA DI COORDINATE
% ==========================================================

fprintf('\n');
fprintf('Sistema di coordinate:\n');

if ~isempty(wkt)

    wktText = char(wkt);

    % Nome CRS
    crsName = regexp(wktText, ...
        'PROJCS\["([^"]+)"', ...
        'tokens', 'once');

    % EPSG del CRS proiettato
    epsgCode = regexp(wktText, ...
        'AUTHORITY\["EPSG","([0-9]+)"\]\]$', ...
        'tokens', 'once');

    % Se non trovato alla fine, cerca l'ultimo EPSG presente
    if isempty(epsgCode)

        allEPSG = regexp(wktText, ...
            'AUTHORITY\["EPSG","([0-9]+)"\]', ...
            'tokens');

        if ~isempty(allEPSG)
            epsgCode = allEPSG{end};
        end

    end

    % Unità del sistema proiettato
    unitMatch = regexp(wktText, ...
        'UNIT\["([^"]+)",[0-9.]+,AUTHORITY\["EPSG","9001"\]\]', ...
        'tokens', 'once');


    if ~isempty(crsName)
        fprintf('  CRS   : %s\n', crsName{1});
    else
        fprintf('  CRS   : Non identificato\n');
    end


    if ~isempty(epsgCode)
        fprintf('  EPSG  : %s\n', epsgCode{1});
    else
        fprintf('  EPSG  : Non disponibile\n');
    end


    if ~isempty(unitMatch)
        fprintf('  Unità : %s\n', unitMatch{1});
    else
        fprintf('  Unità : Non identificata\n');
    end

else

    fprintf('  CRS   : Non disponibile\n');

end


%% =========================================================
% 15. VISUALIZZAZIONE DELLA GRIGLIA ORIGINALE
% ==========================================================

gridPlot = grid;

% Per la sola visualizzazione i NODATA diventano NaN
gridPlot(gridPlot == nodata) = NaN;

fSelect = figure;

imagesc( ...
    [minX maxX], ...
    [minY maxY], ...
    gridPlot);

axis equal;
axis tight;

set(gca, 'YDir', 'normal');

colorbar;

xlabel('X [m]');
ylabel('Y [m]');

title('Elevation Grid - ASC originale');

colormap(parula);


%% =========================================================
% 16. LETTURA DEL FILE ASC
% ==========================================================

filename = ascFile;

% Leggi le prime 6 righe dell'header
fid = fopen(filename, 'r');

if fid == -1
    error('Impossibile aprire il file ASC.');
end

header = cell(6,1);

for i = 1:6
    header{i} = fgetl(fid);
end

fclose(fid);


% Estrai CELLSIZE
CELLSIZE = sscanf(header{5}, 'CELLSIZE %f');

% Calcola il sampling rate [campioni/m]
samplerate = 1 / CELLSIZE;

fprintf('\nCELLSIZE   = %.3f m\n', CELLSIZE);
fprintf('samplerate = %.3f campioni/m\n', samplerate);


%% =========================================================
% 17. LETTURA MATRICE DELLE ELEVAZIONI
% ==========================================================

Zterrain = readmatrix( ...
    filename, ...
    'FileType', 'text', ...
    'NumHeaderLines', 6);

% Correzione orientamento ASC -> MATLAB
Zterrain = flipud(Zterrain);

%% =========================================================
% 18. RIELABORAZIONE DEI NODATA PER TERRAINREPORT
% ==========================================================
%
% L'ASC NON VIENE MODIFICATO.
%
% La matrice Zterrain è una COPIA del contenuto dell'ASC che
% verrà utilizzata esclusivamente da TerrainReport.
%
% Le celle -9999 vengono interpolate utilizzando le quote
% delle celle realmente esistenti.

fprintf('\n============================================\n');
fprintf('INTERPOLAZIONE NODATA PER TERRAINREPORT\n');
fprintf('============================================\n');

maskNoData = (Zterrain == nodata);

numNoDataTerrain = sum(maskNoData(:));

fprintf('Celle NODATA da interpolare: %d\n', ...
    numNoDataTerrain);


if numNoDataTerrain > 0

    % Trasforma temporaneamente i -9999 in NaN
    Zterrain(maskNoData) = NaN;


    % Dimensioni raster
    [nRowsTerrain, nColsTerrain] = size(Zterrain);


    % Coordinate matriciali delle celle
    %
    % Non servono coordinate geografiche assolute per
    % l'interpolazione perché la griglia ha passo uniforme.
    [Xgrid, Ygrid] = meshgrid( ...
        1:nColsTerrain, ...
        1:nRowsTerrain);


    % Celle che possiedono una quota reale
    maskValid = isfinite(Zterrain);


    % Controllo di sicurezza
    if sum(maskValid(:)) < 3

        error(['Numero insufficiente di celle valide ' ...
               'per eseguire l''interpolazione.']);

    end


    % Costruzione dell'interpolante
    %
    % 'natural':
    %   interpolazione morbida all'interno della zona coperta
    %
    % 'nearest':
    %   utilizzato per eventuali celle esterne alla regione
    %   direttamente interpolabile.

    F = scatteredInterpolant( ...
        Xgrid(maskValid), ...
        Ygrid(maskValid), ...
        Zterrain(maskValid), ...
        'natural', ...
        'nearest');


    % Calcola le quote solamente nelle celle che
    % originariamente erano -9999

    Zterrain(maskNoData) = F( ...
        Xgrid(maskNoData), ...
        Ygrid(maskNoData));


end


%% =========================================================
% 19. CONTROLLO DOPO INTERPOLAZIONE
% ==========================================================

numNaNFinali = sum(~isfinite(Zterrain(:)));

fprintf('Celle non valide dopo interpolazione: %d\n', ...
    numNaNFinali);

fprintf('Quota minima dopo interpolazione: %.3f m\n', ...
    min(Zterrain(:)));

fprintf('Quota massima dopo interpolazione: %.3f m\n', ...
    max(Zterrain(:)));

fprintf('============================================\n');


if numNaNFinali > 0

    warning(['Sono ancora presenti celle non valide ' ...
             'dopo l''interpolazione.']);

end


%% =========================================================
% 20. VISUALIZZAZIONE DELLA MATRICE INTERPOLATA
% ==========================================================

figure;

imagesc( ...
    [minX maxX], ...
    [minY maxY], ...
    Zterrain);

axis equal;
axis tight;

set(gca, 'YDir', 'normal');

colorbar;

xlabel('X [m]');
ylabel('Y [m]');

title('Elevation Grid - interpolata per TerrainReport');

colormap(parula);


%% =========================================================
% 21. TRACK WIDTH PER L'ANALISI DI COERENZA
% ==========================================================
%
% TrackWidth indica la distanza laterale tra i profili del DEM
% confrontati nell'analisi di coerenza spaziale. Per un veicolo
% specifico si puo' scegliere la distanza tra le tracce delle ruote.
% Non e' la larghezza della carreggiata esportata in CRG: quella
% viene scelta separatamente durante l'export CRG.
%
% ATTENZIONE: nella versione attuale di TerrainReport_rectangular,
% i vecchi export RDF e PLY riutilizzano ancora TrackWidth per
% definire il ritaglio. Questa dipendenza andra' rimossa quando
% sara' riscritto il blocco RDF/PLY. Cambiare il valore qui, per ora,
% puo' cambiare anche quei due export.
%
% La coerenza lungo X e Y arrotonda la separazione a celle intere:
% TW_ct_XY = round(TrackWidth * samplerate). Il pre-script mostra
% e passa la distanza effettiva corrispondente a quelle celle.
% La coerenza diagonale arrotonda separatamente il suo offset.

[nRowsTW, nColsTW] = size(Zterrain);

if ~isscalar(samplerate) || ~isfinite(samplerate) || samplerate <= 0
    error('samplerate deve essere un valore positivo e finito.');
end

% TerrainReport richiede TW_ct_XY < numero di righe e colonne.
maxTrackCells = min(nRowsTW, nColsTW) - 1;
if maxTrackCells < 1
    error('Il DEM e'' troppo piccolo per scegliere TrackWidth.');
end

demStep = 1 / samplerate;
demExtentX = (nColsTW - 1) * demStep;
demExtentY = (nRowsTW - 1) * demStep;

% Default: al massimo 2 m, e non oltre un quarto del lato minore.
defaultTrackCells = round(min(2.0, ...
    0.25 * min(demExtentX, demExtentY)) * samplerate);
defaultTrackCells = max(1, min(defaultTrackCells, maxTrackCells));
defaultTrackWidth = defaultTrackCells * demStep;

fprintf('\nTRACK WIDTH - COERENZA DEL TERRENO\n');
fprintf('Distanza laterale tra i profili confrontati (es. tracce delle ruote).\n');
fprintf('DEM: %.3f x %.3f m; passo: %.4f m/cella.\n', ...
    demExtentX, demExtentY, demStep);
fprintf('Intervallo utile: %.3f - %.3f m (1 - %d celle).\n', ...
    demStep, maxTrackCells * demStep, maxTrackCells);
fprintf('Nota: gli export RDF/PLY attuali usano ancora TrackWidth.\n');

while true
    twText = strtrim(input(sprintf( ...
        'TrackWidth [%.3f m; Invio = default] > ', ...
        defaultTrackWidth), 's'));

    if isempty(twText)
        trackWidthRequested = defaultTrackWidth;
    else
        trackWidthRequested = str2double(strrep(twText, ',', '.'));
    end

    if ~isscalar(trackWidthRequested) || ...
            ~isfinite(trackWidthRequested) || trackWidthRequested <= 0
        fprintf('Inserisci una distanza positiva e finita.\n');
        continue;
    end

    trackWidthCells = round(trackWidthRequested * samplerate);
    if trackWidthCells < 1 || trackWidthCells > maxTrackCells
        fprintf('La distanza deve corrispondere a 1 - %d celle (%.3f - %.3f m).\n', ...
            maxTrackCells, demStep, maxTrackCells * demStep);
        continue;
    end

    trackWidth = trackWidthCells * demStep;
    break;
end

fprintf('Richiesto: %.3f m; usato per X/Y: %d celle = %.3f m.\n\n', ...
    trackWidthRequested, trackWidthCells, trackWidth);


%% =========================================================
% 21B. CREA LA STRUTTURA RICHIESTA DA TERRAINREPORT
% ==========================================================

GIS.mode = 'real';

GIS.units = 'm';

% IMPORTANTE:
% TerrainReport riceve la matrice interpolata,
% NON la matrice originale contenente -9999.

GIS.Zmatrix = Zterrain;

GIS.TrackWidth = trackWidth;

GIS.samplerate = samplerate;

GIS.CutOff = 10;

GIS.Fext = {};


%% =========================================================
% 22. CONTROLLO
% ==========================================================

fprintf('\nDimensioni Z: %d x %d\n', ...
    size(Zterrain,1), ...
    size(Zterrain,2));

fprintf('Samplerate: %.2f campioni/m\n', ...
    GIS.samplerate);


%% =========================================================
% 23. CONFERMA CONTINUAZIONE
% ==========================================================

risposta = questdlg( ...
    'Vuoi continuare con TerrainReport?', ...
    'Conferma', ...
    'Sì', 'No', 'Sì');


if strcmp(risposta, 'Sì')

    % Minimizza tutte le figure aperte
    figs = findall(groot,'Type','figure');

    for k = 1:numel(figs)
        try
            figs(k).WindowState = 'minimized';
        catch
           
        end
    end
else

    return;

end


%% =========================================================
% 24. AVVIA TERRAINREPORT
% ==========================================================

TerrainReport_rectangular(GIS)
