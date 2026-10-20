function step6_exportAndPlot(grid, minX, maxX, minY, maxY, cellSize, nodata, ascFile, prjFile, wkt, rmseVal)
fid = fopen(ascFile, 'w');
fprintf(fid, 'NCOLS %d\n', size(grid,2));
fprintf(fid, 'NROWS %d\n', size(grid,1));
fprintf(fid, 'XLLCORNER %.6f\n', minX);
fprintf(fid, 'YLLCORNER %.6f\n', minY);
fprintf(fid, 'CELLSIZE %.6f\n', cellSize);
fprintf(fid, 'NODATA_VALUE %.6f\n', nodata);
for r = size(grid,1):-1:1
    fprintf(fid, '%g', grid(r,1));
    fprintf(fid, ' %g', grid(r,2:end));
    fprintf(fid, '\n');
end
fclose(fid);

if ~isempty(wkt)
    fid = fopen(prjFile, 'w');
    fprintf(fid, '%s', wkt);
    fclose(fid);
end

gridPlot = grid; gridPlot(gridPlot == nodata) = NaN;
figure('Name', 'Raster Definitivo', 'Color', 'white');
imagesc([minX maxX], [minY maxY], gridPlot); axis equal; axis tight; set(gca, 'YDir', 'normal'); colorbar;
title(sprintf('Raster Definitivo (RMSE: %.2f mm)', rmseVal*1000)); colormap(parula);
end