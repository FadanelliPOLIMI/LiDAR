function [ptCloudROI] = step7_selectROI(ptCloudIn, config)
    % STEP7_SELECTROI - 2D Top-Down Area of Interest (ROI) Selector
    % Press ENTER to confirm selection and return execution to MAIN.
    
    if nargin < 2 || isempty(config), config = initPipelineConfig(); end
    if ptCloudIn.Count == 0, ptCloudROI = ptCloudIn; return; end
    
    tic;
    
    if ~isfield(config, 'roiExtractionMode'), config.roiExtractionMode = 'interactive'; end
    if ~isfield(config, 'roiType'), config.roiType = 'rectangle'; end % 'rectangle' or 'polygon'
    
    fprintf('\n--------------------------------------------\n');
    fprintf('STAGE 7: 2D TOP-DOWN ROI SELECTOR\n');
    fprintf('--------------------------------------------\n');

    isEdgeMode = isfield(config, 'mode') && strcmpi(config.mode, 'edge');
    hasDisplay = ~isempty(java.awt.GraphicsEnvironment.isHeadless) && ...
                 ~java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment().isHeadless();

    if isEdgeMode || ~hasDisplay
        fprintf('  -> [EDGE/HEADLESS MODE] 2D selector bypassed. Using preset limits or full cloud.\n');
        if isfield(config, 'roiLimits') && ~isempty(config.roiLimits)
            config.roiExtractionMode = 'preset';
        else
            config.roiExtractionMode = 'auto';
        end
    end

    switch lower(config.roiExtractionMode)
        case 'auto'
            ptCloudROI = ptCloudIn;
            
        case 'interactive'
            fprintf('  -> [MODE: INTERACTIVE 2D] Generating top-down road map for selection...\n');
            
            locs = ptCloudIn.Location;
            x = locs(:,1); y = locs(:,2); z = locs(:,3);
            
            % Rasterize quick 2D map
            res = 0.05;
            xEdges = min(x):res:max(x);
            yEdges = min(y):res:max(y);
            
            [~, ~, ~, binX, binY] = histcounts2(x, y, xEdges, yEdges);
            valid = (binX > 0 & binY > 0 & binX <= length(xEdges)-1 & binY <= length(yEdges)-1);
            
            gridZ = nan(length(xEdges)-1, length(yEdges)-1);
            linIdx = sub2ind(size(gridZ), binX(valid), binY(valid));
            gridZ(linIdx) = z(valid);
            
            % Open figure and bind Enter key callback
            fig = figure('Name', 'Stage 7: 2D Top-Down ROI Selector', 'Color', 'w', 'Position', [100, 100, 900, 700], ...
                'WindowKeyPressFcn', @(src, event) handleKeyPress(src, event));
            
            imagesc(xEdges, yEdges, gridZ');
            set(gca, 'YDir', 'normal');
            colormap(gca, jet);
            colorbar;
            axis equal tight;
            grid on;
            
            pos = [];
            
            if strcmpi(config.roiType, 'polygon')
                title({'2D POLYGON SELECTOR', 'Click points to draw shape. Double-click final point OR press ENTER to finish.'}, 'FontSize', 10, 'FontWeight', 'bold');
                h = drawpolygon('Color', 'r', 'LineWidth', 2);
                
                uiwait(fig);
                
                try
                    if isvalid(h)
                        pos = h.Position;
                    end
                    close(fig);
                catch
                    if ishandle(fig), close(fig); end
                end
                
                if ~isempty(pos) && size(pos, 1) >= 3
                    config.roiPolygon = pos;
                    inMask = inpolygon(x, y, pos(:,1), pos(:,2));
                    ptCloudROI = select(ptCloudIn, inMask);
                    fprintf('  -> [2D POLYGON ROI] Extracted %d points inside polygon.\n', ptCloudROI.Count);
                else
                    warning('Polygon selection cancelled or insufficient points. Retaining full cloud.');
                    ptCloudROI = ptCloudIn;
                end
                
            else % Rectangle Mode
                title({'2D RECTANGLE SELECTOR', 'Drag a box over your area, then press ENTER on your keyboard.'}, 'FontSize', 10, 'FontWeight', 'bold');
                h = drawrectangle('Color', 'r', 'LineWidth', 2);
                
                uiwait(fig);
                
                try
                    if isvalid(h)
                        pos = h.Position; % [xmin, ymin, width, height]
                    end
                    close(fig);
                catch
                    if ishandle(fig), close(fig); end
                end
                
                if ~isempty(pos)
                    xmin = pos(1); xmax = pos(1) + pos(3);
                    ymin = pos(2); ymax = pos(2) + pos(4);
                    
                    config.roiLimits = [xmin, xmax, ymin, ymax];
                    inMask = (x >= xmin & x <= xmax & y >= ymin & y <= ymax);
                    ptCloudROI = select(ptCloudIn, inMask);
                    fprintf('  -> [2D RECTANGLE ROI] Extracted %d points inside box.\n', ptCloudROI.Count);
                else
                    warning('Rectangle selection cancelled. Retaining full cloud.');
                    ptCloudROI = ptCloudIn;
                end
            end
            
        case 'preset'
            locs = ptCloudIn.Location;
            if isfield(config, 'roiPolygon') && ~isempty(config.roiPolygon)
                inMask = inpolygon(locs(:,1), locs(:,2), config.roiPolygon(:,1), config.roiPolygon(:,2));
                ptCloudROI = select(ptCloudIn, inMask);
            elseif isfield(config, 'roiLimits') && ~isempty(config.roiLimits)
                lim = config.roiLimits;
                inMask = (locs(:,1) >= lim(1) & locs(:,1) <= lim(2) & ...
                          locs(:,2) >= lim(3) & locs(:,2) <= lim(4));
                ptCloudROI = select(ptCloudIn, inMask);
            else
                ptCloudROI = ptCloudIn;
            end
            
        otherwise
            ptCloudROI = ptCloudIn;
    end
    
    elapsedTime = toc;
    fprintf('  -> Stage 7 ROI Selection Completed in %.2f seconds (Retained: %d pts).\n', ...
        elapsedTime, ptCloudROI.Count);
    fprintf('--------------------------------------------\n');

    % Final visualization preview
    if isfield(config, 'visible') && config.visible && ~isEdgeMode && hasDisplay
        figure('Name', 'Stage 7: Final Selected ROI', 'Color', 'w');
        if isprop(ptCloudROI, 'Color') && ~isempty(ptCloudROI.Color)
            pcshow(ptCloudROI);
        else
            pcshow(ptCloudROI.Location, ptCloudROI.Location(:,3));
            colormap(gca, 'jet');
        end
        title(sprintf('Stage 7 Final ROI (%d points)', ptCloudROI.Count));
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        grid on; drawnow;
    end
end

% --- HELPER FUNCTION: RESUME EXECUTION WHEN ENTER IS PRESSED ---
function handleKeyPress(~, event)
    if strcmpi(event.Key, 'return') || strcmpi(event.Key, 'enter')
        uiresume(gcbf);
    end
end