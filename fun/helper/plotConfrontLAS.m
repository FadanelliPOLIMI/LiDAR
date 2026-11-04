function plotConfrontLAS(file1Path, file2Path)
% PLOTCONFRONTLAS Overlays two LAS/LAZ point clouds on the same plot 
% with two distinct colors for direct visual comparison.

if ~exist(file1Path, 'file')
    error('>>> [ERROR] File 1 not found: %s', file1Path);
end
if ~exist(file2Path, 'file')
    error('>>> [ERROR] File 2 not found: %s', file2Path);
end

fprintf('>>> [COMPARISON] Loading File 1: %s\n', file1Path);
reader1 = lasFileReader(file1Path);
[ptCloud1, ~] = readPointCloud(reader1);

fprintf('>>> [COMPARISON] Loading File 2: %s\n', file2Path);
reader2 = lasFileReader(file2Path);
[ptCloud2, ~] = readPointCloud(reader2);

fprintf('     - File 1 Points: %d\n', ptCloud1.Count);
fprintf('     - File 2 Points: %d\n', ptCloud2.Count);

% Crea la figura di confronto
figure('Name', 'Direct LAS Point Cloud Comparison', 'Position', [100, 100, 1000, 800]);

pts1 = ptCloud1.Location;
pts2 = ptCloud2.Location;

% Plotta File 1 in Blu
scatter3(pts1(:,1), pts1(:,2), pts1(:,3), 5, 'blue', 'filled', 'DisplayName', 'File 1 (Baseline)');
hold on;

% Plotta File 2 in Arancione
scatter3(pts2(:,1), pts2(:,2), pts2(:,3), 5, [0.85 0.32 0.10], 'filled', 'DisplayName', 'File 2 (Reconstructed)');
hold off;

title({'Direct Spatial Comparison Overlay', ...
    sprintf('File 1 (%d pts) vs File 2 (%d pts)', ptCloud1.Count, ptCloud2.Count)}, ...
    'FontSize', 12, 'FontWeight', 'bold');

xlabel('X Projection (m)'); 
ylabel('Y Projection (m)'); 
zlabel('Elevation Z (m)');

legend('Location', 'best', 'FontSize', 10);
grid on; 
axis equal; 
view(3);

fprintf('>>> [SUCCESS] Direct comparison overlay rendered successfully.\n');
end