function plotRaster(Zterrain, config)
    %% =========================================================
    % EXTRA PLOTTING SECTION: FLUID FULL-ROAD INSPECTOR
    % =========================================================
    fprintf('\n--------------------------------------------\n');
    fprintf('EXTRA PLOT: Rendering Fluid Full-Road Inspector\n');
    fprintf('--------------------------------------------\n');
    
    [nrows, ncols] = size(Zterrain);
    gridRes = config.voxelSize;
    
    [X_full, Y_full] = meshgrid((0:ncols-1)*gridRes, (0:nrows-1)*gridRes);
    
    figFluid = figure('Name', 'Fluid Road Surface Inspector', 'Color', 'w', 'Position', [100, 100, 1300, 750]);
    
    hFullSurf = surf(X_full, Y_full, Zterrain);
    set(hFullSurf, 'EdgeColor', 'none');
    
    colormap(turbo);
    colorbar;
    shading interp;
    lightangle(-45, 45);
    lighting gouraud;
    material dull;
    
    % --- THE FIX: USE CONTROLLED STRETCH INSTEAD OF RIGID AXIS EQUAL ---
    % This stretches the 10cm height variance so your eyes can see it,
    % and stretches the 3m width so it's visible, while leaving X free to zoom/pan.
    yStretch = max(1, (ncols*gridRes) / (nrows*gridRes) / 6);
    zStretch = 15; % 15x vertical exaggeration so bumps are actually visible
    daspect([yStretch, 1, 1/zStretch]); 
    
    axis tight;
    box on;
    grid on;
    
    totalLength = ncols * gridRes;
    totalWidth  = nrows * gridRes;
    totalSpan   = max(Zterrain(:)) - min(Zterrain(:));
    
    title(sprintf('Fluid Road Inspector (Length: %.1fm | Width: %.1fm | Z-Span: %.3fm | 15x Z-Stretch)', ...
        totalLength, totalWidth, totalSpan));
    xlabel('Road Length X [m]');
    ylabel('Road Width Y [m]');
    zlabel('Elevation Z [m]');
    
    view(-30, 40);
    
    % --- ACTIVATE ALL BUILT-IN MATLAB INSPECTION TOOLS ---
    % This enables you to click the Pan hand icon, Zoom magnifying glass, 
    % or Rotate 3D tool directly from the figure window toolbar!
    zoom(figFluid, 'on');
    rotate3d(figFluid, 'on');
    pan(figFluid, 'on'); 
    
    fprintf('  -> Fluid inspector ready! Use the figure toolbar icons to Pan, Zoom, or Rotate freely.\n');
end
