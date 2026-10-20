function [ptCloudRaw, wkt, ascFile, prjFile] = step1_loadLAS()
lasFile = uigetfile('*.las', 'Seleziona il file LAS');
if isequal(lasFile, 0), error('Nessun file LAS selezionato.'); end
[~, baseName, ~] = fileparts(lasFile);
ascFile = [baseName '.asc'];
prjFile = [baseName '.prj'];

fprintf('Lettura del file LAS in corso...\n');
reader = lasFileReader(lasFile);
[ptCloudRaw, ~] = readPointCloud(reader);
fprintf('  -> Punti iniziali nel LAS: %d\n', ptCloudRaw.Count);

try
    wktRecord = reader.readVLR(2112);
    wkt = wktRecord.Data;
    fprintf('  -> CRS WKT trovato nel LAS (VLR 2112).\n');
catch
    warning('Il LAS non contiene il VLR 2112. Il file PRJ non verrà creato.');
    wkt = '';
end
end