function config = selectSpatialLimitsInteractive(ptCloud, config)
    % SELECTSPATIALLIMITSINTERACTIVE - Interactive 2D box selector with Zoom/Pan toolbar and OK button
    % Zoom/Pan Toolbar controls, and Dual ROI modes (Rectangle or Polygon for complex curves).
    
    if nargin < 2, config = struct(); end
    if ~isfield(config, 'roiType'), config.roiType = 'rectangle'; end
    
    fprintf('  -> Opening interactive figure for spatial limit selection...\n');
    
    % Enable the figure toolbar to give access to Zoom In, Zoom Out, and Pan tools
    fig = figure('Name', 'Interactive Spatial Limit Selector', ...
        'Color', [0.12 0.12 0.12], ...
        'Position', [100, 100, 1100, 800], ...
        'Toolbar', 'figure');
    
    ax = axes('Parent', fig, 'Color', [0.08 0.08 0.08], ...
        'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    hold(ax, 'on');
    
    locs = ptCloud.Location;
    
    % Downsample points for fast and smooth rendering if point cloud is huge
    maxPointsToDisplay = 500000;
    if size(locs, 1) > maxPointsToDisplay
        idxSample = randperm(size(locs, 1), maxPointsToDisplay);
        locsPlot = locs(idxSample, :);
    else
        locsPlot = locs;
    end
    
    % High-contrast rendering by Z-elevation
    scatter(ax, locsPlot(:,1), locsPlot(:,2), 1.5, locsPlot(:,3), 'filled');
    grid(ax, 'on'); axis(ax, 'equal');
    colormap(ax, 'turbo'); 
    cb = colorbar(ax); cb.Color = 'w'; cb.Label.String = 'Elevation Z [m]';
    
    title(ax, {'INTERACTIVE ROAD CORRIDOR SELECTION', ...
        '• Use Zoom/Pan icons on the figure toolbar to navigate.', ...
        '• Adjust ROI handles to cover your target area.', ...
        '• Click "Confirm Selection (OK)" when done.'}, ...
        'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(ax, 'X [m]', 'Color', 'w'); ylabel(ax, 'Y [m]', 'Color', 'w');
    
    % --- DRAW ROI BASED ON CONFIG ---
    if strcmpi(config.roiType, 'polygon')
        % Polygon Mode for curved roads/intersections
        if isfield(config, 'roiPolygon') && ~isempty(config.roiPolygon)
            roi = drawpolygon(ax, 'Position', config.roiPolygon, 'Color', [1 0.3 0.3], 'LineWidth', 2);
        else
            fprintf('  -> Draw a polygon trace around the curved road corridor. Double-click to complete.\n');
            roi = drawpolygon(ax, 'Color', [1 0.3 0.3], 'LineWidth', 2);
        end
    else
        % Default Rectangle Mode
        hasExistingLimits = all(isfield(config, {'xmin', 'xmax', 'ymin', 'ymax'})) && ...
            ~isempty(config.xmin) && ~isempty(config.xmax);
    
        if hasExistingLimits
            initX = config.xmin;
            initY = config.ymin;
            initW = config.xmax - config.xmin;
            initH = config.ymax - config.ymin;
        else
            xRange = [min(locsPlot(:,1)), max(locsPlot(:,1))];
            yRange = [min(locsPlot(:,2)), max(locsPlot(:,2))];
            initX = xRange(1) + 0.1 * range(xRange);
            initY = yRange(1) + 0.1 * range(yRange);
            initW = 0.8 * range(xRange);
            initH = 0.8 * range(yRange);
        end
        roi = drawrectangle(ax, 'Position', [initX, initY, initW, initH], 'Color', [1 0.3 0.3], 'LineWidth', 2);
    end
    
    % OK Button
    uicontrol('Style', 'pushbutton', 'String', 'Confirm Selection (OK)', ...
        'Position', [460, 20, 180, 40], ...
        'FontSize', 11, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.15 0.65 0.25], 'ForegroundColor', 'w', ...
        'Callback', @(~,~) uiresume(fig));
    
    uiwait(fig);
    
    if ~ishandle(fig)
        error('PipelineError:UserCancelled', 'Selection window was closed without confirmation.');
    end
    
    % Extract ROI data
    if strcmpi(config.roiType, 'polygon')
        config.roiPolygon = roi.Position;
        if size(config.roiPolygon, 1) < 3
            error('PipelineError:InvalidPolygon', 'A polygon ROI requires at least 3 vertices.');
        end
        config.xmin = min(config.roiPolygon(:,1));
        config.xmax = max(config.roiPolygon(:,1));
        config.ymin = min(config.roiPolygon(:,2));
        config.ymax = max(config.roiPolygon(:,2));
    else
        pos = roi.Position; 
        config.xmin = pos(1);
        config.xmax = pos(1) + pos(3);
        config.ymin = pos(2);
        config.ymax = pos(2) + pos(4);
    end
    
    close(fig);
    fprintf('  -> Interactive ROI captured successfully (X: [%.2f, %.2f], Y: [%.2f, %.2f]).\n', ...
        config.xmin, config.xmax, config.ymin, config.ymax);
end