function visualizeDenoising(ptCloudRaw, ptCloudClean)
figure('Name', '3D Comparison: Dense Cloud vs Adaptive Final', 'Color', 'white', 'Position', [100, 100, 1300, 650]);

ax1 = subplot(1, 2, 1);
pcshow(ptCloudRaw, 'Parent', ax1);
title(ax1, sprintf('DENSE Cloud (Original)\nPoints: %d', ptCloudRaw.Count), 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]'); zlabel(ax1, 'Z [m]');
grid(ax1, 'on'); axis(ax1, 'equal'); view(ax1, 3);

ax2 = subplot(1, 2, 2);
pcshow(ptCloudClean, 'Parent', ax2);
title(ax2, sprintf('FINAL Cloud (Propagated)\nPoints: %d', ptCloudClean.Count), 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax2, 'X [m]'); ylabel(ax2, 'Y [m]'); zlabel(ax2, 'Z [m]');
grid(ax2, 'on'); axis(ax2, 'equal'); view(ax2, 3);

linkprop([ax1, ax2], {'CameraPosition', 'CameraTarget', 'CameraUpVector', 'CameraViewAngle'});
fprintf('  -> 3D comparison window successfully opened (Linked views).\n');
end