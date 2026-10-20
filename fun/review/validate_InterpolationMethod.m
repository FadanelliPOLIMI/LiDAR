function [isValid, score, metrics] = validate_InterpolationMethod(ptCloudRoad, Xq, Yq, Z_road, resolution)
% VALIDATE_INTERPOLATIONMETHOD - Valuta la bontà di un metodo di interpolazione
% bilanciando la fedeltà ai punti LiDAR originali e la stabilità dei gradienti per ADAMS.

xyz = double(ptCloudRoad.Location);
x_orig = xyz(:, 1);
y_orig = xyz(:, 2);
z_orig = xyz(:, 3);

% ---------------------------------------------------------------------
% PILASTRO 1: FEDELTÀ AI PUNTI LiDAR ORIGINALI (Verosimiglianza)
% ---------------------------------------------------------------------
% Ricampioniamo la superficie rasterizzata esattamente sulle coordinate 
% (x, y) dei punti LiDAR originali per misurare l'errore di interpolazione.
z_est = interp2(Xq, Yq, Z_road, x_orig, y_orig, 'linear');

validIdx = ~isnan(z_est);
error_z = z_orig(validIdx) - z_est(validIdx);

metrics.RMSE_Fidelity = sqrt(mean(error_z.^2));      % Errore quadratico medio sui punti
metrics.MaxError_Fidelity = max(abs(error_z));      % Errore puntuale massimo

% ---------------------------------------------------------------------
% PILASTRO 2: STABILITÀ DEI GRADIENTI E CURVATURA (Dinamica ADAMS)
% ---------------------------------------------------------------------
[dz_dx, dz_dy] = gradient(Z_road, resolution, resolution);

metrics.MaxSlopeX = max(abs(dz_dx(:)));
metrics.MaxSlopeY = max(abs(dz_dy(:)));

% Calcolo della curvatura approssimata (Laplaciano proxy per stimare i cambi bruschi)
[d2z_dx2, ~] = gradient(dz_dx, resolution, resolution);
[~, d2z_dy2] = gradient(dz_dy, resolution, resolution);
metrics.MaxCurvature = max(abs(d2z_dx2(:) + d2z_dy2(:)));

% ---------------------------------------------------------------------
% LOGICA DI VALIDAZIONE (Soglie ingegneristiche per ADAMS e OpenCRG)
% ---------------------------------------------------------------------
% Criteri di accettazione:
% 1. RMSE rispetto ai punti LiDAR inferiore a 5 cm (0.05m) -> Buona fedeltà geometrica
% 2. Pendenza massima longitudinale e trasversale < 1.0 (circa 45°) -> Niente pareti verticali
% 3. Curvatura controllata -> Evita micro-picchi (coni stile IDW)

condFidelity = metrics.RMSE_Fidelity < 0.05; 
condSlopes   = metrics.MaxSlopeX < 1.0 && metrics.MaxSlopeY < 1.0;
condCurv     = metrics.MaxCurvature < 50.0; % Evita cambi di curvatura folli

isValid = condFidelity && condSlopes && condCurv;

% Calcolo di un punteggio ponderato complessivo (da 0 a 100)
score_fidelity = max(0, 100 * (1 - metrics.RMSE_Fidelity / 0.10)); % 0 RMSE = 100, 10cm RMSE = 0
score_slope    = max(0, 100 * (1 - (metrics.MaxSlopeX + metrics.MaxSlopeY) / 2.0));
score = 0.6 * score_fidelity + 0.4 * score_slope;

% ---------------------------------------------------------------------
% STAMPA REPORT DETTAGLIATO
% ---------------------------------------------------------------------
fprintf('\n====================================================\n');
fprintf('       REPORT VALIDAZIONE METODO INTERPOLAZIONE       \n');
fprintf('====================================================\n');
fprintf('  [1. FEDELTÀ AI DATI ORIGINALI]\n');
fprintf('    * RMSE sui punti LiDAR     : %.4f m (Target < 0.05m)\n', metrics.RMSE_Fidelity);
fprintf('    * Errore Max Puntuale      : %.4f m\n', metrics.MaxError_Fidelity);
fprintf('  [2. STABILITÀ PER MSC ADAMS]\n');
fprintf('    * Pendenza Max Long. (X)   : %.2f rad (%.1f°)\n', metrics.MaxSlopeX, rad2deg(atan(metrics.MaxSlopeX)));
fprintf('    * Pendenza Max Trasp. (Y)  : %.2f rad (%.1f°)\n', metrics.MaxSlopeY, rad2deg(atan(metrics.MaxSlopeY)));
fprintf('    * Curvatura Massima        : %.2f\n', metrics.MaxCurvature);
fprintf('  [3. ESITO DELLA COUNTER-ACTION]\n');
if isValid
    fprintf('    * STATO: SUPERATO (Metodo validato per OpenCRG/ADAMS)\n');
else
    fprintf('    * STATO: FALLITO (Rischio instabilità in ADAMS o perdita cuneette)\n');
end
fprintf('    * PUNTEGGIO COMPLESSIVO    : %.1f / 100\n', score);
fprintf('====================================================\n');
end