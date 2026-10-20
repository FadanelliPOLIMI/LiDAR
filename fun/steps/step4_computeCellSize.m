function cellSize = step4_computeCellSize(X_orig, Y_orig, Z_orig, xMin, yMin, xMax, yMax, config)
aoiWidth = xMax - xMin; aoiHeight = yMax - yMin; aoiArea = aoiWidth * aoiHeight;
pointDensity = numel(Z_orig) / aoiArea;
cellSizeLAS = sqrt(config.targetPtsPerCell / pointDensity);
cellSizePercent = min(aoiWidth, aoiHeight) * (config.resPercent / 100);

cellSizeRaw = max(cellSizePercent, cellSizeLAS);
cellSize = ceil(cellSizeRaw / config.cellSizeQuantum) * config.cellSizeQuantum;
cellSize = max(cellSize, config.cellSizeQuantum);

fprintf('\n--- ANALISI GEOMETRICA AOI ---\n');
fprintf('Dimensioni: %.3f x %.3f m | Densità: %.3f pt/m^2 | CellSize: %.3f m\n', ...
    aoiWidth, aoiHeight, pointDensity, cellSize);
end