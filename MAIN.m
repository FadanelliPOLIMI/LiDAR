%% DATA PROCESSING %%
clear; clc;
% Aggiunge la cartella 'fun' e sottocartelle al percorso di ricerca di MATLAB
addpath('fun');
addpath('files/las');
addpath('fun/interpolate');
addpath('fun/steps');
addpath('fun/helper');
%% PARAMETRI DI CONFIGURAZIONE GLOBALE
config.voxelSize        = 0.05;   % [m] Risoluzione Voxel Grid (2 cm)
config.resPercent       = 0.25;   % [%] Percentuale del lato minore dell'AOI
config.targetPtsPerCell = 4;      % Punti LAS medi desiderati per cella
config.cellSizeQuantum  = 0.001;  % Quantizzazione finale cella [m] (1 mm)
config.nodata           = -9999;  % Valore standard per celle vuote
config.rmseThreshold    = 0.005;  % [m] Soglia di tolleranza RMSE (5 mm)
config.benchmark        = true;   % for benchmark cfr
config.optimize         = false;  % for bayesian AT optimization
config.compareInterp    = false;   % compara metodi di interpolazione
config.optimizeInterp   = false;   % ottimizza parametri interpolazione
config.fillingHoles     = true;   % fills holes before interpolating
config.sliceWidth       = 0.5;    % slioce width filling holes
config.gaussFilt        = true;  % Gaussian Filter
%% 0. CREAZIONE LAS CON RGB E CORRETTAMENTE GEOREFERENZIATO
% Nome della cartella dei dati
dataFolder = 'files/las';
% Lista dei file LAS che vuoi elaborare in sequenza
fileList = {
    'Alta Frequenza 5 cm.las',
    %'Bassa Frequenza 5cm.las', % Aggiungi qui altri file se necessario
    };

% Parametri globali opzionali (se vuoi applicarli a tutti i file)
paramName = 'MinHeightLimit';
paramVal  = 0.10;

% 3. Ciclo di elaborazione batch
disp(['>>> [BATCH] Avvio elaborazione di ', num2str(length(fileList)), ' file LiDAR...']);

for i = 1:length(fileList)
    currentFile = fullfile(dataFolder, fileList{i});
    fprintf('\n------------------------------------------------------------\n');
    fprintf('>>> Elaborazione file %d di %d: %s\n', i, length(fileList), currentFile);
    fprintf('------------------------------------------------------------\n');

    try
        % Chiamata alla funzione georefPipeline creata in precedenza
        [~, savedPath] = step0_georefPipeline(currentFile, paramName, paramVal);
        fprintf(' >>> [COMPLETATO] File elaborato con successo: %s\n', savedPath);
    catch ME
        fprintf(' >>> [ERRORE] Impossibile completare il file %s.\n', currentFile);
        fprintf(' >>> Dettaglio errore: %s\n', ME.message);
        % Continua con il file successivo anche se uno fallisce
        continue; 
    end
end

disp('\n============================================================');
disp('>>> [BATCH COMPLETATO] Tutti i file in lista sono stati processati!');
disp('============================================================');

%% 1. LETTURA LAS E WKT
[ptCloudRaw, wkt, ascFile, prjFile] = step1_loadLAS();

%% 2A. DENOISING STATISTICO (SOR)
[X_clean, Y_clean, Z_clean, ptCloudClean] = step2a_denoiseSOR(ptCloudRaw);

%% 2B. CONTROLLO ADATTIVO PRE-ML
% Sfoltiamo e uniformiamo la nuvola all'inizio usando la risoluzione configurata.
% Questo alleggerisce drasticamente la RAM per il Machine Learning e la PCA.
% La funzione valuta l'RMSE e sceglie se usare i punti sfoltiti o densi anche a
% seconda delle deviazioni e della densita
fprintf('  -> Esecuzione controllo adattivo preventivo sulla nuvola pulita...\n');
[X_work, Y_work, Z_work, rmseVal, maxErrorVal, ptCloudWork] = step2b_adaptiveVoxelCheck( ...
    X_clean, Y_clean, Z_clean, config, ptCloudClean, false);
fprintf('     Punti ridotti da %d a %d per ottimizzazione di sistema.\n', ptCloudClean.Count, ptCloudWork.Count);


%% 3. ESTRAZIONE STRADA INTERATTIVA (Scelta Modulo)%% 
paramsFile = 'bestParameters_cache.mat';
if isfield(config, 'benchmark') && config.benchmark
    % =========================================================================
    % BAYESIAN AUTO-TUNING OPTIMIZATION PARAMS
    % =========================================================================
    % Controlliamo se l'utente vuole ricalcolarli o caricare la cache esistente
    if isfield(config, 'optimize') && ~config.optimize && exist(paramsFile, 'file')
        fprintf('  -> Caricamento parametri ottimali pre-salvati da: %s\n', paramsFile);
        loadedData = load(paramsFile, 'bestParameters');
        bestParameters = loadedData.bestParameters;
    else
        fprintf('\n>>> Ottimizzazione Bayesiana per AutoTuner...\n');
        variables = [
            optimizableVariable('binSize', [0.05, 0.5], 'Transform', 'log')
            optimizableVariable('macroSize', [15.0, 35.0])       % Specifico Pro2
            optimizableVariable('minSearchHeight', [0.05, 0.25])
            optimizableVariable('K1', [10, 25], 'Type', 'integer')
            optimizableVariable('K2', [40, 80], 'Type', 'integer')
            optimizableVariable('maxNumSplits', [20, 60], 'Type', 'integer')
            optimizableVariable('greenThreshold', [0.05, 0.60]) 
            ];
        
        results = bayesopt(@(p) costFunctionAT(p, ptCloudRaw), variables, ...
            'MaxObjectiveEvaluations', 40, 'UseParallel', true, 'Verbose', 0);
        bestParameters = bestPoint(results);
        save(paramsFile, 'bestParameters');
        fprintf('  -> Parametri ottimali salvati con successo in: %s\n', paramsFile);
    end
    % =========================================================================
    % BENCHMARK COMPARATIVO MULTI-METOD: Random Forest vs Deep Learning vs Active Learning
    % =========================================================================
    fprintf('>>> Avvio benchmark comparativo tra i 3 metodi di estrazione strada...\n');

    if ~exist('ptCloudWork', 'var')
        ptCloudWork = ptCloudRaw; 
    end

    % --- 1. ESECUZIONE METODO 3A: Random Forest Classico ---
    fprintf('\n--- [1/3] Esecuzione Random Forest (3A) ---\n');
    tic;
    try
        [ptCloudRoad_RF] = step3a_RandomForest(ptCloudWork);
    catch ME
        warning('Errore in Random Forest: %s. Creo nuvola vuota di fallback.', ME.message);
        ptCloudRoad_RF = pointCloud([0 0 0]);
    end
    timeRF = toc;

    % --- 2. Esecuzione METODO 3B: Deep Learning 3D (RandLA-Net) ---
    fprintf('\n--- [2/3] Esecuzione Deep Learning 3D RandLA-Net(3B) ---\n');
    tic;
    try
        [ptCloudRoad_DL] = step3b_DLRandLANet(ptCloudWork);
    catch ME
        warning('Errore in Deep Learning: %s. Creo nuvola vuota di fallback.', ME.message);
        ptCloudRoad_DL = pointCloud([0 0 0]);
    end
    timeDL = toc;

    % --- 3. Esecuzione METODO 3C: Active Learning / Pro2 ---
    fprintf('\n--- [3/3] Esecuzione Active Learning Pro2 (3C) ---\n');
    tic;
    try
        [ptCloudRoad_AL] = step3c_autoTunerFilter(ptCloudWork, 'min', bestParameters.binSize,...
            bestParameters.macroSize,'road', bestParameters.minSearchHeight, bestParameters.K1,...
            bestParameters.K2, bestParameters.maxNumSplits, bestParameters.greenThreshold, false);
    catch ME
        warning('Errore in Active Learning: %s. Creo nuvola vuota di fallback.', ME.message);
        ptCloudRoad_AL = pointCloud([0 0 0]);
    end
    timeAL = toc;

    % --- METRICHE QUANTITATIVE ---
    xyz_rf = double(ptCloudRoad_RF.Location); 
    if ~isempty(xyz_rf), zMAD_rf = median(abs(xyz_rf(:,3) - median(xyz_rf(:,3)))); else, zMAD_rf = 0; end

    xyz_dl = double(ptCloudRoad_DL.Location); 
    if ~isempty(xyz_dl), zMAD_dl = median(abs(xyz_dl(:,3) - median(xyz_dl(:,3)))); else, zMAD_dl = 0; end

    xyz_al = double(ptCloudRoad_AL.Location); 
    if ~isempty(xyz_al), zMAD_al = median(abs(xyz_al(:,3) - median(xyz_al(:,3)))); else, zMAD_al = 0; end

    % --- TABELLA RISULTATI A TERMINALE ---
    fprintf('\n======================== RISULTATI BENCHMARK COMPARATIVO ========================\n');
    fprintf('METRICA                      | RANDOM FOREST (3A) | DEEP LEARNING (3B) | ACTIVE LEARNING (3C)\n');
    fprintf('---------------------------------------------------------------------------------\n');
    fprintf('Tempo di Esecuzione          | %6.2f sec        | %6.2f sec        | %6.2f sec        \n', timeRF, timeDL, timeAL);
    fprintf('Punti Estratti (Strada)      | %8d           | %8d           | %8d           \n', ptCloudRoad_RF.Count, ptCloudRoad_DL.Count, ptCloudRoad_AL.Count);
    fprintf('Percentuale sulla Nuvola     | %6.1f%%           | %6.1f%%           | %6.1f%%           \n', (ptCloudRoad_RF.Count/ptCloudWork.Count)*100, (ptCloudRoad_DL.Count/ptCloudWork.Count)*100, (ptCloudRoad_AL.Count/ptCloudWork.Count)*100);
    fprintf('Planarità Asfalto (MAD Z)    | %6.4f m          | %6.4f m          | %6.4f m          \n', zMAD_rf, zMAD_dl, zMAD_al);
    fprintf('=================================================================================\n');

    % --- E. PLOT COMPARATIVO A 3 VIE (Gradato per altezza e interattivo) ---
    fprintf('  -> Generazione confronto visivo tridimensionale a tre vie per MSC Adams...\n');
    figBench = figure('Name', 'Benchmark Comparativo: RF (3A) vs DL (3B) vs AL (3C)', 'Color', 'w', 'Position', [50, 100, 1500, 600]);

    % Subplot 1: Random Forest (Gradato Z)
    ax1 = subplot(1, 3, 1);
    if ptCloudRoad_RF.Count > 1
        xyz_rf = ptCloudRoad_RF.Location;
        pcshow(xyz_rf, xyz_rf(:,3), 'MarkerSize', 10, 'Parent', ax1);
        colormap(ax1, jet);
    end
    title(sprintf('3A: Random Forest\nPunti: %d | MAD: %.4fm', ptCloudRoad_RF.Count, zMAD_rf), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    grid on; axis equal; view(3);

    % Subplot 2: Deep Learning (Gradato Z)
    ax2 = subplot(1, 3, 2);
    if ptCloudRoad_DL.Count > 1
        xyz_dl = ptCloudRoad_DL.Location;
        pcshow(xyz_dl, xyz_dl(:,3), 'MarkerSize', 10, 'Parent', ax2);
        colormap(ax2, jet);
    end
    title(sprintf('3B: Deep Learning\nPunti: %d | MAD: %.4fm', ptCloudRoad_DL.Count, zMAD_dl), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    grid on; axis equal; view(3);

    % Subplot 3: Active Learning Pro2 (Gradato Z)
    ax3 = subplot(1, 3, 3);
    if ptCloudRoad_AL.Count > 1
        xyz_al = ptCloudRoad_AL.Location;
        pcshow(xyz_al, xyz_al(:,3), 'MarkerSize', 10, 'Parent', ax3);
        colormap(ax3, jet);
    end
    title(sprintf('3C: Active Learning Pro2\nPunti: %d | MAD: %.4fm', ptCloudRoad_AL.Count, zMAD_al), 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    grid on; axis equal; view(3);

    % Rende la finestra interattiva e non bloccante per il workspace
    drawnow;
    fprintf('>>> Benchmark e ottimizzazione completati con successo! (Puoi ruotare e zoomare i grafici 3D)\n');
end

fprintf('\n============================================\n');
fprintf('SELEZIONE INTERATTIVA METODO ESTRAZIONE STRADA\n');
fprintf('============================================\n');

% Finestra di dialogo grafica per la scelta dell'utente
choice = questdlg('Quale approccio desideri utilizzare per isolare la strada?', ...
    'Selezione Metodo Estrazione', ...
    'Random Forest (3A)', 'Deep Learning RandLANET(3B)', 'Active Learning (3C)', 'Random Forest (3A)');
% Esecuzione condizionata in base alla scelta dell'utente
switch choice
    % 3A MACHINE LEARNING CLASSICO (Random Forest)
    % Addestramento del modello + Applicazione per isolare la strada e rimuovere la vegetazione
    case 'Random Forest (3A)'
        fprintf('  -> Avvio Approccio 3A: Random Forest Classico (Parallelo)...\n');
        [ptCloudRoad] = step3a_RandomForest(ptCloudWork);
        % 3B. DEEP LEARNING 3D (RandLA-Net)
    case 'Deep Learning RandLA-Net(3B)'
        fprintf('  -> Avvio Approccio 3B: Deep Learning 3D (RandLA-Net)...\n');
        [ptCloudRoad] = step3b_DLRandLANet(ptCloudWork);
        % 3C AUTO-ENCODING (Active Learning & Pseudo-Labeling)
    case 'Active Learning (3C)'
        fprintf('  -> Avvio Approccio 3C: Active Learning & Valley Detection Pro...\n');
        [ptCloudRoad] = step3c_autoTunerFilter(ptCloudWork);
        % 3D. DEEP LEARNING 3D (PointNet++)
    case 'Deep Learning PointNet++(3D)'
        fprintf('  -> Avvio Approccio 3D: Deep Learning 3D (PointNet++)...\n');
        % PROVA POINTENET++
        % % 1. Ottieni la cartella in cui si trova attualmente lo script MAIN
        % currentDir = pwd; % oppure mfilename('fullpath') se lo script è una funzione
        % % 2. Fai 4 passi indietro nella gerarchia delle cartelle
        % [parentDir1, ~] = fileparts(currentDir);
        % [parentDir2, ~] = fileparts(parentDir1);
        % [parentDir3, ~] = fileparts(parentDir2);
        % [baseDir,    ~] = fileparts(parentDir3); % Questa è la cartella di arrivo dopo 4 passi indietro
        % % 3. Da lì, scendi nella sottocartella LiDAR/PointNet++
        % datafolder = fullfile(baseDir, 'LiDAR', 'PointNet++');
        % % Verifica che il percorso esista effettivamente
        % if isfolder(datafolder)
        %     fprintf('Percorso impostato con successo: %s\n', datafolder);
        % else
        %     warning('Attenzione: La cartella non esiste: %s', datafolder);
        % end
        % dataFolder = fullfile('LiDAR', 'PointNet++');
        % trainDataFolder = fullfile(dataFolder,'dales_las','train');
        % testDataFolder = fullfile(dataFolder,'dales_las','test');
        % lasReader = lasFileReader(fullfile(trainDataFolder,'5080_54435.las'));
        % [pc,attr] = readPointCloud(lasReader,'Attributes','Classification');
        % labels = attr.Classification;
        % % Select only labeled data.
        % pc = select(pc,labels~=0);
        % labels = labels(labels~=0);
        % classNames = [
        %     "ground"
        %     "vegetation"
        %     "cars"
        %     "trucks"
        %     "powerlines"
        %     "fences"
        %     "poles"
        %     "buildings"
        %     ];
        % figure;
        % ax = pcshow(pc.Location,labels);
        % helperLabelColorbar(ax,classNames);
        % title("Point Cloud with Overlaid Semantic Labels");
    otherwise
        error('Processo interrotto: nessun metodo selezionato o finestra chiusa dall''utente.');
end

fprintf('--> [SUCCESSO] Variabile ptCloudRoad definita con successo: %d punti stradali isolati.\n', ptCloudRoad.Count);

%% 4. ESTRAZIONE AUTOMATICA DELLA STRADA (Con controllo colore integrato)
fprintf('Estrazione automatica del terreno e della strada...\n');
[X_orig, Y_orig, Z_orig, xMin, yMin, xMax, yMax, ptCloudRoadAligned] = step4_alignRoadPCA(ptCloudRoad);
fprintf('  -> Area stradale rilevata automaticamente: X[%.1f, %.1f], Y[%.1f, %.1f]\n', ...
    xMin, xMax, yMin, yMax);

%% 5A. INPAINTING GEOMETRICO DEI BUCHI STRADALI
% Assicurati di definire il parametro nel blocco di configurazione iniziale:
% config.fillingHoles = true; % (o false per saltarlo)
% config.sliceWidth = 0.5;    % Ampiezza della fetta in metri

if isfield(config, 'fillingHoles') && config.fillingHoles
    fprintf('  -> [CONFIG] Inpainting geometrico attivato.\n');

    % Se vuoi usare la versione bidirezionale che abbiamo fatto prima, 
    % togli i controlli interni di config dalla funzione e lasciala snella.
    [ptCloudReadyForInterp, addedPointsCount] = step5b_gapsDiagnostics(ptCloudRoadAligned, config.sliceWidth);

else
    fprintf('  -> [CONFIG] Inpainting disattivato da configurazione. Salto il passaggio.\n');
    ptCloudReadyForInterp = ptCloudRoadAligned;
    addedPointsCount = 0;
end

%% 5B. LAST GAUSSIAN FILTER (CANCELLA COLORI E INTENSITA)
if isfield(config, 'gaussFilt') && config.gaussFilt
    fprintf('  -> [CONFIG] Gaussian filter attivato: loss of color and intensity data.\n');
    ptc = gaussFilt(ptCloudReadyForInterp);
end

%% 6. INTERPOLAZIONE E RASTERIZZAZIONE FINALE DELLA SUPERFICIE STRADALE
% =========================================================================
fprintf('\n============================================\n');
fprintf('ELABORAZIONE SUPERFICIE E INTERPOLAZIONE (OTTIMIZZATA RAM-SAFE)\n');
fprintf('============================================\n');

% Controllo variabili di configurazione e fallback di sicurezza
if ~isfield(config, 'voxelSize'), config.voxelSize = 0.05; end
if ~isfield(config, 'compareInterp'), config.compareInterp = false; end
if ~isfield(config, 'optimizeInterp'), config.optimizeInterp = false; end

interpParamsFile = 'bestInterpParameters_cache.mat';
methodsList = {'bspl', 'spatial', 'spectral', 'tpaps', 'idw', 'mls', 'krg', 'rbf', 'st_opencrg'};

% Gestione del Pool Parallelo Sicuro (Limitato a max 4 worker per proteggere la RAM)
poolObj = gcp('nocreate');
if isempty(poolObj) && (config.optimizeInterp || config.compareInterp)
    try
        numWorkers = min(4, feature('numcores'));
        poolObj = parpool('local', numWorkers);
        fprintf('  -> Pool parallelo avviato con %d worker (RAM-Safe).\n', numWorkers);
    catch
        warning('Impossibile avviare il pool parallelo. L''elaborazione proseguirà in seriale.');
    end
end

% ---------------------------------------------------------
% A. GESTIONE PARAMETRI (Ottimizzazione Bayesiana Parametri IDW & MLS)
% ---------------------------------------------------------
if config.optimizeInterp
    fprintf('\n>>> Ottimizzazione Bayesiana parametri interpolazione (IDW p & MLS Radius) in corso...\n');
    
    variablesInterp = [
        optimizableVariable('idwPower', [0.5, 4.0], 'Type', 'real') ...
        optimizableVariable('searchRadiusMLS', [0.2, 2.0], 'Type', 'real')
    ];
    
    costFcn = @(p) localCostFunctionInterpolation(p, ptCloudRoadAligned, config.voxelSize);
    
    resultsInterp = bayesopt(costFcn, variablesInterp, ...
        'MaxObjectiveEvaluations', 15, ...
        'UseParallel', ~isempty(poolObj), ...
        'Verbose', 0);
        
    bestInterpParams = bestPoint(resultsInterp);
    save(interpParamsFile, 'bestInterpParams');
    fprintf('  -> Parametri di interpolazione salvati con successo in: %s\n', interpParamsFile);
    fprintf('     * IDW Power (p) ottimale: %.3f\n', bestInterpParams.idwPower);
    fprintf('     * MLS Search Radius ottimale: %.3f m\n', bestInterpParams.searchRadiusMLS);
    
elseif exist(interpParamsFile, 'file')
    fprintf('  -> Caricamento parametri di interpolazione ottimali da cache: %s\n', interpParamsFile);
    loadedInterpData = load(interpParamsFile, 'bestInterpParams');
    bestInterpParams = loadedInterpData.bestInterpParams;
else
    bestInterpParams = struct('idwPower', 2.0, 'searchRadiusMLS', 0.6);
end

% ---------------------------------------------------------
% B. SELEZIONE METODO: BENCHMARK PARALLELO SICURO SUI 9 METODI
% ---------------------------------------------------------
if config.compareInterp
    fprintf('\n>>> Modalità Benchmark Interpolazioni Attiva (Confronto Parallelo su 9 Metodi - High Fidelity)...\n');
    
    numMethods = length(methodsList);
    zMAD_interp = zeros(numMethods, 1);
    timeInterp  = zeros(numMethods, 1);
    Z_results   = cell(numMethods, 1);
    X_results   = cell(numMethods, 1);
    Y_results   = cell(numMethods, 1);
    
    % Copia locale dei dati per il trasferimento sicuro nel parfor
    ptCloudInput = ptCloudRoadAligned;
    vSize = config.voxelSize;
    params = bestInterpParams;
    
    % Esecuzione in Parallelo del Benchmark sui 9 Metodi
    parfor m = 1:numMethods
        mName = methodsList{m};
        tLocal = tic;
        Z_q = []; X_q = []; Y_q = [];
        
        try
            switch lower(mName)
                case 'spatial'
                    [Z_q, X_q, Y_q, ~] = interpolate_Spatial_Optimal(ptCloudInput, vSize);
                case 'spectral'
                    [X_q, Y_q, Z_q, ~] = interpolate_Spectral_FFT(ptCloudInput, vSize);
                case 'tpaps'
                    [X_q, Y_q, Z_q] = interpolate_ThinPlateSpline(ptCloudInput, vSize);
                case 'idw'
                    [X_q, Y_q, Z_q] = interpolate_IDW(ptCloudInput, vSize, params.idwPower);
                case 'mls'
                    [X_q, Y_q, Z_q] = interpolate_MovingLeastSquares(ptCloudInput, vSize, params.searchRadiusMLS);
                case 'krg'
                    [X_q, Y_q, Z_q] = interpolate_UniversalKriging(ptCloudInput, vSize);
                case 'bspl'
                    [X_q, Y_q, Z_q] = interpolate_BSplineSurface(ptCloudInput, vSize);
                case 'rbf'
                    [X_q, Y_q, Z_q] = interpolate_RBF(ptCloudInput, vSize);
                case 'st_opencrg'
                    [X_q, Y_q, Z_q, ~] = interpolate_ST_OpenCRG(ptCloudInput, vSize);
            end
        catch ME
            warning('Metodo [%s] fallito (%s).', mName, ME.message);
        end
        
        timeInterp(m) = toc(tLocal);
        Z_results{m} = Z_q;
        X_results{m} = X_q;
        Y_results{m} = Y_q;
        
        if ~isempty(Z_q)
            validZ = Z_q(~isnan(Z_q));
            if ~isempty(validZ)
                zMAD_interp(m) = median(abs(validZ - median(validZ)));
            else
                zMAD_interp(m) = NaN;
            end
        else
            zMAD_interp(m) = NaN;
        end
    end
    
    % ---------------------------------------------------------
    % RENDERING VISUALE 1: Griglia 3x3 delle Colormap 2D
    % ---------------------------------------------------------
    figInterpBench = figure('Name', 'Benchmark Completo Interpolazioni - Colormaps (9 Metodi)', ...
        'Color', 'w', 'Position', [40, 40, 1600, 900]);
        
    for m = 1:numMethods
        ax = subplot(3, 3, m);
        Z_q = Z_results{m};
        if ~isempty(Z_q)
            imagesc(ax, Z_q); axis equal tight; colorbar; colormap(ax, jet);
            title(sprintf('[%s]\nMAD: %.4fm | Tempo: %.2fs', upper(methodsList{m}), zMAD_interp(m), timeInterp(m)), ...
                'FontSize', 9, 'FontWeight', 'bold');
            xlabel('Cols (px)'); ylabel('Rows (px)');
        else
            title(sprintf('[%s] - FALLITO', upper(methodsList{m})), 'Color', 'r', 'FontSize', 9, 'FontWeight', 'bold');
        end
    end
    drawnow;

    % ---------------------------------------------------------
    % RENDERING VISUALE 2: Gallerie 3D Affiancate per il Confronto Geometrico
    % ---------------------------------------------------------
    fig3DCompare = figure('Name', 'Confronto Geometrico 3D - Tutti i Metodi di Interpolazione', ...
        'Color', 'w', 'Position', [100, 100, 1700, 950]);
    
    for m = 1:numMethods
        ax3d = subplot(3, 3, m, 'Parent', fig3DCompare);
        Z_q = Z_results{m};
        X_q = X_results{m};
        Y_q = Y_results{m};
        
        if ~isempty(Z_q) && ~isempty(X_q) && ~isempty(Y_q)
            if isvector(X_q) && isvector(Y_q) && length(X_q) == size(Z_q, 2) && length(Y_q) == size(Z_q, 1)
                [Xmesh, Ymesh] = meshgrid(X_q, Y_q);
            else
                Xmesh = X_q; Ymesh = Y_q;
            end
            
            surf(ax3d, Xmesh, Ymesh, Z_q, 'EdgeColor', 'none');
            shading(ax3d, 'interp');
            lighting(ax3d, 'gouraud');
            camlight(ax3d, 'headlight');
            
            title(ax3d, sprintf('[%s] (%.2fs)', upper(methodsList{m}), timeInterp(m)), 'FontSize', 9, 'FontWeight', 'bold');
            xlabel(ax3d, 'X (m)'); ylabel(ax3d, 'Y (m)'); zlabel(ax3d, 'Z (m)');
            view(ax3d, 45, 30);
            grid(ax3d, 'on');
        else
            text(ax3d, 0.5, 0.5, 0.5, 'Dati non disponibili', 'HorizontalAlignment', 'center');
            title(ax3d, sprintf('[%s]', upper(methodsList{m})));
        end
    end
    drawnow;
    
    fprintf('>>> Benchmark interpolazioni e grafici comparativi completati con successo!\n');
    
    % Selezione automatica del metodo vincitore
    [~, bestIdx] = min(zMAD_interp(~isnan(zMAD_interp)));
    if isempty(bestIdx), bestIdx = 1; end
    interpolationMethod = methodsList{bestIdx};
    fprintf('--> Selezione automatica vincitrice dal benchmark: [%s]\n', upper(interpolationMethod));
else
    % Selezione Interattiva tramite GUI
    [selectionIdx, ok] = listdlg('PromptString', 'Scegli il metodo di interpolazione per la superficie stradale:', ...
        'SelectionMode', 'single', ...
        'ListString', upper(methodsList), ...
        'InitialValue', 2, ...
        'Name', 'Selezione Interpolazione MSC Adams', ...
        'ListSize', [300, 200]);
    
    if ok && ~isempty(selectionIdx)
        interpolationMethod = methodsList{selectionIdx};
    else
        error('Processo interrotto: Nessun metodo di interpolazione selezionato.');
    end
end

% ---------------------------------------------------------
% C. ESECUZIONE FINALE CON IL METODO SELEZIONATO
% ---------------------------------------------------------
fprintf('\n  -> Calcolo superficie definitiva con metodo: [%s] (Passo: %.3fm)...\n', ...
    upper(interpolationMethod), config.voxelSize);

switch lower(interpolationMethod)
    case 'spatial'
        [Z_road, Xq, Yq, gridStruct] = interpolate_Spatial_Optimal(ptCloudRoadAligned, config.voxelSize);
    case 'spectral'
        [Xq, Yq, Z_road, ~] = interpolate_Spectral_FFT(ptCloudRoadAligned, config.voxelSize);
    case 'tpaps'
        [Xq, Yq, Z_road] = interpolate_ThinPlateSpline(ptCloudRoadAligned, config.voxelSize);
    case 'idw'
        [Xq, Yq, Z_road] = interpolate_IDW(ptCloudRoadAligned, config.voxelSize, bestInterpParams.idwPower);
    case 'mls'
        [Xq, Yq, Z_road] = interpolate_MovingLeastSquares(ptCloudRoadAligned, config.voxelSize, bestInterpParams.searchRadiusMLS);
    case 'krg'
        [Xq, Yq, Z_road] = interpolate_UniversalKriging(ptCloudRoadAligned, config.voxelSize);
    case 'bspl'
        [Xq, Yq, Z_road] = interpolate_BSplineSurface(ptCloudRoadAligned, config.voxelSize);
    case 'rbf'
        [Xq, Yq, Z_road] = interpolate_RBF(ptCloudRoadAligned, config.voxelSize);
    case 'st_opencrg'
        [Xq, Yq, Z_road, ~] = interpolate_ST_OpenCRG(ptCloudRoadAligned, config.voxelSize);
    otherwise
        [Z_road, Xq, Yq, gridStruct] = interpolate_Spatial_Optimal(ptCloudRoadAligned, config.voxelSize);
end

fprintf('  -> Matrice della superficie stradale calcolata correttamente (%dx%d nodi).\n', ...
    size(Z_road, 1), size(Z_road, 2));






























interpolationMethod = 'bspl'; % Opzioni: 'spatial', 'spectral', 'tpaps', 'idw', 'mls'

switch lower(interpolationMethod)
    case 'spatial'
        [Xq, Yq, Z_road] = interpolate_Spatial_Standard(ptCloudRoadOnly, 0.1);
    case 'spectral'
        [Xq, Yq, Z_road, wavelength] = interpolate_Spectral_FFT(ptCloudRoadOnly, 0.1);
    case 'tpaps'
        [Xq, Yq, Z_road] = interpolate_ThinPlateSpline(ptCloudRoadOnly, 0.1);
    case 'idw'
        [Xq, Yq, Z_road] = interpolate_IDW(ptCloudRoadOnly, 0.1, 2);
    case 'mls'
        [Xq, Yq, Z_road] = interpolate_MovingLeastSquares(ptCloudRoadOnly, 0.1, 0.6);
    case 'krg'
        [Xq, Yq, Z_road] = interpolate_Spatial_Standard(ptCloudRoadOnly, 0.1);
    case 'bspl'
        [Xq, Yq, Z_road] = interpolate_BSplineSurface(ptCloudRoadOnly, 0.1);
    case 'rbf'
        [Xq, Yq, Z_road] = interpolate_RBF(ptCloudRoadOnly, 0.1);
    case 'st_opencrg'
        [Xq, Yq, Z_road, ~] = interpolate_ST_OpenCRG(ptCloudRoadOnly, 0.1);
        % Nota: per s-t, Xq e Yq saranno Sq e Tq
end








%%









































% =========================================================
% PARAMETRI GRIGLIA E RASTER (Semplificati e Deterministici)
% =========================================================
% Il cellSize viene ereditato direttamente dalla configurazione del Voxel Check
cellSize = config.voxelSize; % es. 0.05 m (5 cm)
nodata   = -9999;

% Calcolo immediato dimensioni griglia sulla nuvola filtrata (X_final, Y_final, Z_final)
minX = floor(min(X_final)); maxX = ceil(max(X_final));
minY = floor(min(Y_final)); maxY = ceil(max(Y_final));

ncols = ceil((maxX - minX) / cellSize);
nrows = ceil((maxY - minY) / cellSize);

fprintf('--> Griglia Raster deterministica creata: %d x %d celle (Passo: %.3f m)\n', ...
    ncols, nrows, cellSize);



%% 3. INTERPOLAZIONE, GESTIONE INCERTEZZA E MONTE CARLO (La nostra pipeline Master)
[optimalRoadGrid, reportTable, stochasticVariations] = run_MasterBayesianRoadPipeline(ptCloudRoad, 0.1);