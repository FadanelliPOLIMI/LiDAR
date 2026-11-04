function report = inspectLasFile(lasFile)
% INSPECTLASFILE Inspects a LAS/LAZ file, checking for coordinates,
% CRS, intensity, RGB colors, and general data integrity.

fprintf('\n============================================================\n');
fprintf('>>> START FILE INSPECTION: %s\n', lasFile);
fprintf('============================================================\n');

report.fileName = lasFile;
report.isValid = false;
report.hasCoordinates = false;
report.hasCRS = false;
report.hasIntensity = false;
report.hasRGB = false;
report.pointCount = 0;

if ~isfile(lasFile)
    error('The specified file does not exist: %s', lasFile);
end

try
    % 1. Open reader and get basic metadata (works for both .las and .laz)
    reader = lasFileReader(lasFile);
    report.pointCount = reader.Count;
    fprintf('[OK] Total declared points: %d\n', report.pointCount);

    if report.pointCount == 0
        warning('[WARNING] The point cloud file is empty (0 points).');
        return;
    end

    % 2. Check 3D Coordinates (X, Y, Z)
    [ptCloud, ~] = readPointCloud(reader);
    if ~isempty(ptCloud.Location) && size(ptCloud.Location, 2) == 3
        report.hasCoordinates = true;
        fprintf('[OK] 3D coordinates (X, Y, Z) found. MinZ: %.2f | MaxZ: %.2f\n', ...
            min(ptCloud.Location(:,3)), max(ptCloud.Location(:,3)));
    else
        fprintf('[ERR] 3D coordinates are missing or invalid.\n');
    end

    % 3. Check Coordinate Reference System (CRS / WKT)
    try
        wktRecord = reader.readVLR(2112);
        if ~isempty(wktRecord) && ~isempty(wktRecord.Data)
            report.hasCRS = true;
            report.wkt = wktRecord.Data;
            fprintf('[OK] Coordinate Reference System (CRS / WKT VLR 2112) found.\n');
        else
            fprintf('[WARN] VLR 2112 exists but is empty. No CRS associated.\n');
        end
    catch
        fprintf('[WARN] No CRS (VLR 2112) found in the file.\n');
    end

    % 4. Check Intensity Channel
    if isprop(ptCloud, 'Intensity') && ~isempty(ptCloud.Intensity)
        report.hasIntensity = true;
        fprintf('[OK] INTENSITY channel present (Range: %d - %d).\n', ...
            min(ptCloud.Intensity), max(ptCloud.Intensity));
    else
        fprintf('[INFO] Intensity channel not present.\n');
    end

    % 5. Check RGB Color Channels
    if isprop(ptCloud, 'Color') && ~isempty(ptCloud.Color)
        report.hasRGB = true;
        fprintf('[OK] RGB color channels present in the point cloud.\n');
    else
        fprintf('[INFO] RGB color channels not present.\n');
    end

    report.isValid = true;
    fprintf('============================================================\n');
    fprintf('>>> INSPECTION COMPLETED SUCCESSFULLY\n');
    fprintf('============================================================\n');

catch ME
    fprintf('[CRITICAL ERROR] Failed to read point cloud file: %s\n', ME.message);
end
end