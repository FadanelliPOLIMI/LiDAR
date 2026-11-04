function [ptCloudColored, outputLasFile] = colorizeAndExport(ctx, shiftEast, shiftNorth, varargin)
% COLORIZEANDEXPORT Applies shifts, colorizes points, embeds WKT, and saves output.
%
% Optional Parameters:
%   'Visible' - boolean (true/false) to control final 3D inspection figures display. Default: true.

p = inputParser;
addParameter(p, 'Visible', true, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
config = p.Results;

% Determine figure visibility state string for MATLAB handles
if config.Visible
    figVisState = 'on';
else
    figVisState = 'off';
end

disp('>>> [STEP] Applying final georeferencing shift and saving LAS file...');

deltaLat_final = shiftNorth / ctx.metersPerLat;
deltaLon_final = shiftEast / ctx.metersPerLon;
lat_final = ctx.lat_orig + deltaLat_final;
lon_final = ctx.lon_orig + deltaLon_final;
[x_new, y_new] = projfwd(ctx.projCRSData, lat_final, lon_final);

validMask = ~isnan(lat_final) & ~isnan(lon_final);
normLat_adj = (lat_final - ctx.latMin) / (ctx.latMax - ctx.latMin);
normLon_adj = (lon_final - ctx.lonMin) / (ctx.lonMax - ctx.lonMin);
pxPoints = round(normLon_adj * (ctx.imgWidth - 1)) + 1;
pyPoints = round((1 - normLat_adj) * (ctx.imgHeight - 1)) + 1;
inImg = validMask & pxPoints >= 1 & pxPoints <= ctx.imgWidth & pyPoints >= 1 & pyPoints <= ctx.imgHeight;
pointColors = uint8(ones(length(lat_final), 3) * 128); 
validIdxs = find(inImg);
for k = 1:length(validIdxs)
    idx = validIdxs(k);
    pointColors(idx, :) = ctx.satImg(pyPoints(idx), pxPoints(idx), :);
end

updatedLocations = [x_new, y_new, ctx.ptCloud.Location(:, 3)];
ptCloudColored = pointCloud(updatedLocations, 'Color', pointColors, 'Intensity', ctx.ptCloud.Intensity);

% =========================================================================
% SAVE INTO PARALLEL RESULTS FOLDER (files/las/results)
% =========================================================================
if ~isfolder(ctx.resultsDir)
    mkdir(ctx.resultsDir);
    fprintf('>>> Created results directory: %s\n', ctx.resultsDir);
end

outputLasFilename = [ctx.fileBaseName, '_Georeferenced.las'];
outputLasFile = fullfile(ctx.resultsDir, outputLasFilename);

try
    writer = lasFileWriter(outputLasFile);
    % Robust WKT Injection using reader or context
    wktString = '';
    try
        vlrRec = ctx.lasReader.readVLR(2112);
        if ~isempty(vlrRec) && ~isempty(vlrRec.Data)
            wktString = vlrRec.Data;
        end
    catch
        % Fallback if reader doesn't have it or field is missing
    end

    % Inject using your centralized helper function
    insertWKT(writer, wktString);
    writePointCloud(writer, ptCloudColored);
    disp([' >>> [SUCCESS] Georeferenced LAS file saved to: ', outputLasFile]);
catch ME
    outputPlyFilename = [ctx.fileBaseName, '_Georeferenced.ply'];
    outputLasFile = fullfile(ctx.currentDir, outputPlyFilename);
    pcwrite(ptCloudColored, outputLasFile, 'Encoding', 'binary');
    disp([' >>> [Notice] Saved in PLY format. LAS Error: ', ME.message]);
end

% Final 3D Inspection Figure (controlled by 'Visible' option)
disp('>>> Generating final 3D inspection figures...');
figure('Name', 'Georeferencing Analysis - 3D Global & Terrain Inspection', ...
    'Position', [50, 50, 1400, 700], ...
    'Visible', figVisState);

subplot(1, 2, 1);
pcshow(ptCloudColored);
title({'Georeferenced 3D LiDAR Point Cloud (Updated)', 'Colorized from Satellite Basemap'}, 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Projected X (m)'); ylabel('Projected Y (m)'); zlabel('Elevation Z (m)');
grid on;

subplot(1, 2, 2);
groundMask = ~ctx.anomalyIdx; 
groundPtsXYZ = ctx.ptCloud.Location(groundMask, :);
scatter3(groundPtsXYZ(:,1), groundPtsXYZ(:,2), groundPtsXYZ(:,3), 12, groundPtsXYZ(:,3), 'filled');
colormap(parula);
colorbar;
title({'3D Terrain Surface Inspection', '(Checking elevation continuity & anomalies)'}, 'FontSize', 11, 'FontWeight', 'bold');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Elevation Z (m)');
grid on;
box on;
axis equal;
view(3); 

disp('>>> [Pipeline Completed Successfully!]');
end