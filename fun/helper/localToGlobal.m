function [X_global, Y_global] = localToGlobal(X_local, Y_local, geoTransform)
% LOCALTOGLOBAL - Maps local PCA/translated coordinates back to global CRS
local_shifted = [X_local + geoTransform.xOffset, Y_local + geoTransform.yOffset];
global_coords = (local_shifted * geoTransform.R) + geoTransform.centroid;
X_global = global_coords(:, 1);
Y_global = global_coords(:, 2);
end