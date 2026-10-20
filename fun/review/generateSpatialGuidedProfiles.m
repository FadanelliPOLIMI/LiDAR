function [nominalGrid, subsidenceGrid, upliftGrid, macroUncertaintyReport] = generateSpatialGuidedProfiles(ptCloudGround, resolution)
% GENERATESPATIALGUIDEDPROFILES - Genera 3 profili stradali continui (Nominale, 
% Cedimento, Rialzo) basati su macro-deformazioni guidate dall'incertezza Bayesiana.
% Ideale per analisi DoE ridotta in MSC Adams senza corrompere le derivate OpenCRG.

fprintf('\n----------------------------------------------------\n');
fprintf(' ESTRAZIONE MACRO-DEFORMAZIONI SPAZIALI (Karhunen-Loève style)\n');
fprintf('----------------------------------------------------\n');

% 1. Calcolo della griglia nominale e della mappa di incertezza Bayesiana
[Xq, Yq, Z_mean, Z_std] = interpolate_BayesianKrigingRisk(ptCloudGround, resolution, 1.96);

% 2. Eliminazione del rumore bianco locale (Smoothing spaziale forte)
% Usiamo un filtro gaussiano ad ampio raggio per estrarre solo le onde lunghe 
% e i trend di cedimento strutturale a bassa frequenza.
sigmaWindowSize = round(5.0 / resolution); % Finestra basata su 5 metri di trend spaziale
if mod(sigmaWindowSize, 2) == 0, sigmaWindowSize = sigmaWindowSize + 1; end

H = fspecial('gaussian', [sigmaWindowSize sigmaWindowSize], sigmaWindowSize / 3);
Z_std_macro = imfilter(Z_std, H, 'replicate', 'same');

% Normalizzazione della macro-incertezza per scalare correttamente la deformazione
maxMacroUnc = max(Z_std_macro(:));
fprintf(' -> Incertezza macroscopica massima stimata: %.4f metri\n', maxMacroUnc);

% 3. Generazione del DoE a 3 profili calibrati
% Profilo Nominale (Superficie stimata principale)
nominalGrid.Xq = Xq;
nominalGrid.Yq = Yq;
nominalGrid.Z  = Z_mean;
nominalGrid.Name = 'Nominal_Profile';

% Profilo di Cedimento Strutturale (- Macro-Varianza guidata)
% Sprofondamento continuo e liscio proporzionale all'incertezza strutturale
subsidenceGrid.Xq = Xq;
subsidenceGrid.Yq = Yq;
subsidenceGrid.Z  = Z_mean - Z_std_macro;
subsidenceGrid.Name = 'Structural_Subsidence_WorstCase';

% Profilo di Rialzo Geometrico (+ Macro-Varianza guidata)
% Sollevamento continuo e liscio proporzionale all'incertezza strutturale
upliftGrid.Xq = Xq;
upliftGrid.Yq = Yq;
upliftGrid.Z  = Z_mean + Z_std_macro;
upliftGrid.Name = 'Geometric_Uplift_WorstCase';

% Report sintetico per il tracciamento ingegneristico
macroUncertaintyReport = struct();
macroUncertaintyReport.MeanUncertainty = mean(Z_std(:));
macroUncertaintyReport.MaxMacroUncertainty = maxMacroUnc;
macroUncertaintyReport.SmoothingWindowMeters = 5.0;

fprintf(' -> [OK] Generati con successo i 3 profili di macro-deformazione continua.\n');
fprintf(' -> Pronto per il packaging OpenCRG e le 3 simulazioni in MSC Adams.\n');
fprintf('----------------------------------------------------\n');
end

%[appendix]{"version":"1.0"}
%---
