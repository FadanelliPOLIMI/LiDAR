function plotPtCloud3D(ptCloudIn, colorMode)
% PLOTPTCLOUD3D Visualizes a 3D point cloud with flexible coloring options
% and interactive distance measurement (Hold Shift + Click to measure).
%
% SYNTAX:
%   plotPtCloud3D(ptCloudIn, colorMode) 
%   colorMode options: 'height' (default), 'intensity', 'rgb'

    if nargin < 2 || isempty(colorMode)
        colorMode = 'height'; 
    end
    
    if isempty(ptCloudIn) || ptCloudIn.Count == 0
        warning('Point cloud is empty. Skipping visualization.');
        return;
    end
    
    fig = figure('Name', '3D Point Cloud Viewer & Distance Measurement', 'Color', 'w');
    
    % Handle MATLAB pointCloud objects natively
    if isa(ptCloudIn, 'pointCloud')
        switch lower(colorMode)
            case 'rgb'
                if ~isempty(ptCloudIn.Color)
                    pcshow(ptCloudIn);
                else
                    warning('No RGB color found. Falling back to Z-height coloring.');
                    pcshow(ptCloudIn.Location, ptCloudIn.Location(:,3));
                    colormap('turbo');
                end
            case 'intensity'
                if ~isempty(ptCloudIn.Intensity)
                    pcshow(ptCloudIn.Location, double(ptCloudIn.Intensity));
                    colormap('hot'); % Hot colormap highlights intensity reflectivity well
                else
                    warning('No Intensity found. Falling back to Z-height coloring.');
                    pcshow(ptCloudIn.Location, ptCloudIn.Location(:,3));
                    colormap('turbo');
                end
            otherwise % Default to Z-height gradient
                pcshow(ptCloudIn.Location, ptCloudIn.Location(:,3));
                colormap('turbo');
        end
    else
        % Fallback if passing raw [N x 3] coordinate matrices
        locs = double(ptCloudIn);
        scatter3(locs(:,1), locs(:,2), locs(:,3), 2, locs(:,3), 'filled');
        colormap('turbo');
    end
    
    % Aesthetics and layout
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('Point Cloud Viewer [%s Mode] (Total Points: %d)\n*Hold Shift + Click to measure distance*', upper(colorMode), ptCloudIn.Count));
    
    if ~strcmpi(colorMode, 'rgb')
        cb = colorbar;
        cb.Label.String = sprintf('Scalar Value (%s)', colorMode);
    end
    
    view(3); % Default 3D orbital view
    axis equal;
    
    % --- ENABLE INTERACTIVE DISTANCE MEASUREMENT ---
    dcm = datacursormode(fig);
    set(dcm, 'UpdateFcn', @(obj, eventObj) pointCloudDistanceCallback(obj, eventObj));
    set(dcm, 'DisplayStyle', 'datatip');
end

function txt = pointCloudDistanceCallback(~, eventObj)
    % Custom Data Cursor callback to calculate Euclidean distance between points
    pos = get(eventObj, 'Position');
    
    % Access all active data tips in the figure
    fig = ancestor(eventObj.Target, 'figure');
    dcm = datacursormode(fig);
    cursorInfo = getCursorInfo(dcm);
    
    % Base coordinate text
    txt = {
        sprintf('X: %.3f m', pos(1)), ...
        sprintf('Y: %.3f m', pos(2)), ...
        sprintf('Z: %.3f m', pos(3))
    };
    
    % If multiple points are selected (Shift + Click), compute distance to previous point
    if length(cursorInfo) > 1
        prevPos = cursorInfo(2).Position; % Preceding point
        euclDist = norm(pos - prevPos);
        dx = abs(pos(1) - prevPos(1));
        dy = abs(pos(2) - prevPos(2));
        dz = abs(pos(3) - prevPos(3));
        
        txt{end+1} = '---';
        txt{end+1} = sprintf('ΔX: %.3f m', dx);
        txt{end+1} = sprintf('ΔY: %.3f m', dy);
        txt{end+1} = sprintf('ΔZ: %.3f m', dz);
        txt{end+1} = sprintf('Distance: %.3f m', euclDist);
    else
        txt{end+1} = '---';
        txt{end+1} = '(Hold Shift + Click next point)';
    end
end