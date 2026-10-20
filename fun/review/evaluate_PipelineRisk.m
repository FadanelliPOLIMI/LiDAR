function [bestMethod, riskTable] = evaluate_PipelineRisk(ptCloudRoadOnly, resolution)
methodsList = {'spatial', 'tpaps', 'kriging', 'bspline', 'rbf', 'st_opencrg'};
numMethods = length(methodsList);

riskTable = table();

% Pesi di importanza (dettati dalle priorità della simulazione)
w_geom = 0.45;  % Massima priorità: la dinamica veicolare non tollera errori geometrici
w_time = 0.20;  % Efficienza temporale
w_data = 0.15;  % Leggerezza per la trasmissione
w_pkg  = 0.20;  % Facilità di packaging OpenCRG

for i = 1:numMethods
    mName = methodsList{i};
    fprintf('Valutazione indice di rischio per il metodo: %s...\n', mName);

    tStart = tic;
    % Esecuzione condizionale del metodo
    try
        switch mName
            case 'spatial', [Xq, Yq, Z] = interpolate_Spatial_Optimized(ptCloudRoadOnly, resolution);
            case 'tpaps',   [Xq, Yq, Z] = interpolate_ThinPlateSpline(ptCloudRoadOnly, resolution);
            case 'kriging', [Xq, Yq, Z, ~] = interpolate_UniversalKriging(ptCloudRoadOnly, resolution);
            case 'bspline', [Xq, Yq, Z] = interpolate_BSplineSurface(ptCloudRoadOnly, resolution);
            case 'rbf',     [Xq, Yq, Z] = interpolate_RBF(ptCloudRoadOnly, resolution);
            case 'st_opencrg',[Xq, Yq, Z, ~] = interpolate_ST_OpenCRG(ptCloudRoadOnly, resolution);
        end
        execTime = toc(tStart);
        failed = false;
    catch
        execTime = 999;
        failed = true;
        R_total = 1.0; % Rischio massimo per fallimento algoritmico
    end

    if ~failed
        % 1. Calcolo Rischio Geometrico (usando la nostra validazione)
        [isValid, ~, metrics] = validate_InterpolationMethod(ptCloudRoadOnly, Xq, Yq, Z, resolution);
        R_geom = min(1.0, metrics.RMSE_Fidelity / 0.10) + (metrics.MaxSlopeX > 1.0)*0.5;
        R_geom = min(1.0, R_geom);

        % 2. Calcolo Rischio Tempo (assumendo 30s come limite massimo accettabile)
        R_time = min(1.0, execTime / 30.0);

        % 3. Calcolo Rischio Dati (basato sulla dimensione della matrice Z in MB)
        whosZ = whos('Z');
        sizeMB = whosZ.bytes / (1024^2);
        R_data = min(1.0, sizeMB / 50.0); % 50MB come soglia critica di trasmissione

        % 4. Rischio Packaging (assegnato in base allo standard industriale)
        if strcmp(mName, 'st_opencrg'), R_pkg = 0.1;
        elseif ismember(mName, {'spatial', 'bspline'}), R_pkg = 0.3;
        else, R_pkg = 0.6; end

        % Calcolo Indice Complessivo OPR
        R_total = (w_geom * R_geom) + (w_time * R_time) + (w_data * R_data) + (w_pkg * R_pkg);
    end

    % Salvataggio in tabella
    newRow = {mName, R_total, R_geom, execTime, R_pkg};
    riskTable = [riskTable; newRow];
end

riskTable.Properties.VariableNames = {'Method', 'OPR_Index', 'GeomRisk', 'TimeSec', 'PkgRisk'};

% Selezione automatica del metodo con il rischio complessivo minore (Min-Risk Solution)
[minRisk, bestIdx] = min(riskTable.OPR_Index);
bestMethod = riskTable.Method{bestIdx};

fprintf('\n========================================\n');
fprintf(' RISULTATO ANALISI DI RISCHIO OPERATIVO \n');
fprintf('========================================\n');
disp(riskTable);
fprintf('--> La soluzione meno rischiosa è: %s (OPR = %.4f)\n', bestMethod, minRisk);
fprintf('========================================\n');
end