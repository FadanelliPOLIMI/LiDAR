function [X_clean, Y_clean, Z_clean, ptCloudClean] = step2a_denoiseSOR(ptCloudRaw)
fprintf('Applicazione Denoising Statistico (SOR)...\n');
[ptCloudClean, ~, outlierIdx] = pcdenoise(ptCloudRaw, 'NumNeighbors', 15, 'Threshold', 1.5);
fprintf('  -> Punti di rumore rimossi: %d (%.1f%%)\n', length(outlierIdx), (length(outlierIdx)/ptCloudRaw.Count)*100);

X_clean = double(ptCloudClean.Location(:,1));
Y_clean = double(ptCloudClean.Location(:,2));
Z_clean = double(ptCloudClean.Location(:,3));

valid = isfinite(X_clean) & isfinite(Y_clean) & isfinite(Z_clean);
X_clean = X_clean(valid); Y_clean = Y_clean(valid); Z_clean = Z_clean(valid);
end