function [optimalGrid, riskReport, mcRealizations] = run_MasterBayesianRoadPipeline(ptCloudGround, resolution)
% RUN_MASTERBAYESIANROADPIPELINE - Pipeline Master End-to-End
% Unisce Machine Learning spaziale, Incertezza Bayesiana, Monte Carlo e 
% Selezione Automatica del Rischio (OPR) per OpenCRG / MSC ADAMS.

    if nargin < 2, resolution = 0.1; end

    fprintf('\n====================================================\n');
    fprintf('  AVVIO PIPELINE MASTER BAYESIANA & MONTE CARLO ADAMS \n');
    fprintf('====================================================\n');

    % -----------------------------------------------------------------
    % FASE 1: CANDIDATE MODEL EVALUATION (Machine Learning & Geostatistics)
    % -----------------------------------------------------------------
    methodsList = {'spatial', 'tpaps', 'kriging', 'bspline', 'rbf', 'st_opencrg'};
    numMethods = length(methodsList);
    riskTable = table();
    storedGrids = struct();

    w_geom = 0.35;  % Pesi del rischio operativo (OPR)
    w_time = 0.15;
    w_data = 0.15;
    w_pkg  = 0.15;
    w_unc  = 0.20;  % Peso specifico dell'incertezza Bayesiana/Monte Carlo

    for i = 1:numMethods
        mName = methodsList{i};
        fprintf('\n[Test Model %d/%d] Esecuzione algoritmo: %s...\n', i, numMethods, upper(mName));
        
        tStart = tic;
        try
            switch mName
                case 'spatial',    [Xq, Yq, Z] = interpolate_Spatial_Optimized(ptCloudGround, resolution);
                case 'tpaps',      [Xq, Yq, Z] = interpolate_ThinPlateSpline(ptCloudGround, resolution);
                case 'kriging',    [Xq, Yq, Z, ~] = interpolate_UniversalKriging(ptCloudGround, resolution);
                case 'bspline',    [Xq, Yq, Z] = interpolate_BSplineSurface(ptCloudGround, resolution);
                case 'rbf',        [Xq, Yq, Z] = interpolate_RBF(ptCloudGround, resolution);
                case 'st_opencrg', [Xq, Yq, Z, ~] = interpolate_ST_OpenCRG(ptCloudGround, resolution);
            end
            execTime = toc(tStart);
            failed = false;
        catch ME
            fprintf('  -> Errore nell''algoritmo %s: %s\n', mName, ME.message);
            execTime = 999;
            failed = true;
        end

        if failed
            R_total = 1.0;
            storedGrids.(mName) = [];
        else
            % Validazione geometrica (Gradienti e Fedeltà)
            [isValid, ~, metrics] = validate_InterpolationMethod(ptCloudGround, Xq, Yq, Z, resolution);
            R_geom = min(1.0, metrics.RMSE_Fidelity / 0.10) + (metrics.MaxSlopeX > 1.0)*0.5;
            R_geom = min(1.0, R_geom);
            
            % Rischio Tempo ed Dati
            R_time = min(1.0, execTime / 30.0);
            whosZ = whos('Z');
            R_data = min(1.0, (whosZ.bytes / (1024^2)) / 50.0);
            
            % Rischio Packaging OpenCRG
            if strcmp(mName, 'st_opencrg'), R_pkg = 0.1;
            elseif ismember(mName, {'spatial', 'bspline'}), R_pkg = 0.3;
            else, R_pkg = 0.6; end

            % ---------------------------------------------------------
            % FASE 2: BAYESIAN UNCERTAINTY QUANTIFICATION & OPR SCORING
            % ---------------------------------------------------------
            % Stimiamo l'incertezza locale con un processo gaussiano leggero sui dati
            [~, ~, ~, riskMap] = interpolate_BayesianKrigingRisk(ptCloudGround, resolution, 1.96);
            R_uncertainty = mean(riskMap(:)); % Incertezza media normalizzata

            % Calcolo OPR (Operational Pipeline Risk Index)
            R_total = (w_geom * R_geom) + (w_time * R_time) + ...
                      (w_data * R_data) + (w_pkg * R_pkg) + (w_unc * R_uncertainty);
            
            storedGrids.(mName).Xq = Xq;
            storedGrids.(mName).Yq = Yq;
            storedGrids.(mName).Z  = Z;
        end

        newRow = {mName, R_total, R_geom, execTime, R_pkg};
        riskTable = [riskTable; newRow];
    end

    riskTable.Properties.VariableNames = {'Method', 'OPR_Index', 'GeomRisk', 'TimeSec', 'PkgRisk'};

    % -----------------------------------------------------------------
    % FASE 3: AUTOMATED SELECTION (Scelta automatica del minor rischio)
    % -----------------------------------------------------------------
    [minRisk, bestIdx] = min(riskTable.OPR_Index);
    bestMethod = riskTable.Method{bestIdx};

    fprintf('\n====================================================\n');
    fprintf('         RISULTATO AUTOMAZIONE DELLA SCELTA         \n');
    fprintf('====================================================\n');
    disp(riskTable);
    fprintf('--> Algoritmo selezionato da MATLAB: %s (OPR Minimo = %.4f)\n', upper(bestMethod), minRisk);
    fprintf('====================================================\n');

    optimalGrid = storedGrids.(bestMethod);

    % -----------------------------------------------------------------
    % FASE 4: MONTE CARLO STOCHASTIC REALIZATIONS (Stress-Test per ADAMS)
    % -----------------------------------------------------------------
    fprintf('\n[Monte Carlo Engine] Generazione di N=50 varianti stocastiche...\n');
    fprintf('  -> Applicazione della varianza posteriore Bayesiana per lo stress-test ADAMS...\n');
    
    [~, ~, Z_mean, Z_std] = interpolate_BayesianKrigingRisk(ptCloudGround, resolution, 1.96);
    numRealizations = 50;
    mcRealizations = cell(numRealizations, 1);
    
    for k = 1:numRealizations
        % Perturbazione stocastica coerente con la mappa di incertezza sigma
        noiseField = randn(size(Z_mean)) .* Z_std;
        mcRealizations{k} = Z_mean + noiseField;
    end
    
    riskReport = riskTable;
    fprintf('[Pipeline Master] Completata con successo! Pronto per l''export in OpenCRG.\n');
end

Come usare la nuova pipeline nel tuo script:

Ti basterà una sola riga di comando per far girare l'intera intelligenza artificiale ingegneristica:
Matlab

% Esegui l'intera architettura Bayesiana-Monte Carlo sulla nuvola pulita
[optimalRoadGrid, reportTable, stochasticVariations] = run_MasterBayesianRoadPipeline(ptCloudRoadOnly, 0.1);

% Visualizza la superficie ottimale scelta da MATLAB
figure;
surf(optimalRoadGrid.Xq, optimalRoadGrid.Yq, optimalRoadGrid.Z, 'EdgeColor', 'none');
title(sprintf('Superficie Stradale Ottimizzata - OpenCRG Ready'));
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
colorabar; colormap jet;

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
