function [ptCloudColored, outputLasFile] = georefPipeline(lasName, config, varargin)
% GEOREFPIPELINE Master orchestrator with selectable alignment mode.
% 1. Prepare data, rasters, and config context

[ctx, config] = prepareDataAndRaster(lasName, config, varargin{:});

% 2. Select alignment path based on user preference
if strcmpi(config.alignmentMode, 'manual')
    disp('>>> [Alignment Mode] MANUAL selected: Launching interactive adjustment GUI.');
    [shiftEast, shiftNorth] = runManualGUI(ctx);
else
    disp('>>> [Alignment Mode] AUTO selected: Running automated DL -> HOG -> imregcorr stack.');
    [shiftEast, shiftNorth] = alignPointClouds(ctx, config);
end

% 3. Colorize points, embed WKT, and export final file
[ptCloudColored, outputLasFile] = colorizeAndExport(ctx, shiftEast, shiftNorth,'visible',false);
end