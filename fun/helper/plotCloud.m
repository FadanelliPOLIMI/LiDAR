function plotCloud(ptCloudIn, data)
% PLOTEXG Visualizes a 3D point cloud colored by its Excess Green (ExG) feature.
%   plotExG(ptCloudIn, exg) takes a point cloud object and an ExG vector, 
%   then generates a 3D scatter plot colored via the 'turbo' colormap.

if isempty(ptCloudIn) || ptCloudIn.Count == 0
    warning('Point cloud is empty. Skipping visualization.');
    return;
end

% Extract spatial coordinates
locs = double(ptCloudIn.Location);

% Create visualization window
figure('Name', 'ExG Feature Visualization', 'Color', 'w');
scatter3(locs(:,1), locs(:,2), locs(:,3), 3, data, 'filled');

% Configure plot aesthetics and layout
axis equal;
grid on;
colormap('turbo');  

cb = colorbar;
cb.Label.String = 'Excess Green (ExG) Value';

title('LiDAR Point Cloud - ExG Feature Map');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

% Default to a top-down planar view for ground/vegetation analysis
view(2); 
end