function [ptCloudCleanCluster] = extractLargestCluster(ptCloudIn, distThresh)
    % EXTRACTLARGESTCLUSTER - Retains the primary contiguous road surface
    
    if nargin < 2 || isempty(distThresh), distThresh = single(0.20); end
    if ptCloudIn.Count == 0
        ptCloudCleanCluster = ptCloudIn;
        return;
    end
    
    searchRadius = single(distThresh);
    
    try
        [labelsCluster, numClusters] = pcsegdist(ptCloudIn, searchRadius);
        if numClusters > 1
            clusterSizes = histcounts(labelsCluster, 1:(numClusters+1));
            [~, largestClusterIdx] = max(clusterSizes);
    
            cleanIndices = (labelsCluster == largestClusterIdx);
            ptCloudCleanCluster = select(ptCloudIn, cleanIndices);
    
            fprintf('  -> Cluster Isolation: Retained main body (%d pts). Removed %d detached points.\n', ...
                ptCloudCleanCluster.Count, ptCloudIn.Count - ptCloudCleanCluster.Count);
        else
            ptCloudCleanCluster = ptCloudIn;
            fprintf('  -> Cluster Isolation: Single continuous body confirmed (%d pts).\n', ptCloudIn.Count);
        end
    catch ME
        warning('Cluster isolation failed (%s). Returning input.', ME.message);
        ptCloudCleanCluster = ptCloudIn;
    end
    end