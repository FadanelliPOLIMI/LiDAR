function [grid, minX, maxX, minY, maxY, ncols, nrows] = step5_rasterizeGrid(...
    X_in, Y_in, Z_in, xMin, yMin, xMax, yMax, cellSize, config)

fprintf('\n============================================\n');
fprintf('CREAZIONE GRIGLIA DEM PER OPENCRG / ADAMS\n');
fprintf('============================================\n');

minX = floor(min(X_in)); 
maxX = ceil(max(X_in));
minY = floor(min(Y_in)); 
maxY = ceil(max(Y_in));

ncols = ceil((maxX - minX) / cellSize);
nrows = ceil((maxY - minY) / cellSize);

fprintf('  -> Dimensioni griglia: %d colonne x %d righe (CellSize: %.3f m)\n', ncols, nrows, cellSize);

colIdx = floor((X_in - minX) / cellSize) + 1;
rowIdx = floor((Y_in - minY) / cellSize) + 1;

validPts = colIdx >= 1 & colIdx <= ncols & rowIdx >= 1 & rowIdx <= nrows;

grid = accumarray(...
    [rowIdx(validPts), colIdx(validPts)], ...
    Z_in(validPts), ...
    [nrows ncols], ...
    @mean, ...
    config.nodata);

fprintf('  -> Griglia DEM rasterizzata con successo.\n');
fprintf('============================================\n');
end