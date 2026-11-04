function insertWKT(writer, wkt)
% INJECTWKTIFNEEDED Checks if WKT is available and injects VLR 2112 into the LAS writer.

if ~isempty(wkt) && ischar(wkt) || isstring(wkt)
    wktStr = char(wkt);
    if ~isempty(strtrim(wktStr))
        try
            % Compatibilità con i metodi nativi di MATLAB per la scrittura VLR
            if ismethod(writer, 'addVLR')
                writer.addVLR(2112, 'LASF_Projection', wktStr, 'OGC Coordinate System WKT');
            else
                addVLR(writer, 2112, 'LASF_Projection', wktStr, 'OGC Coordinate System WKT');
            end
            fprintf('     - [CRS] WKT successfully injected into VLR 2112.\n');
            return;
        catch ME
            warning('>>> [CRS] Failed to inject VLR 2112: %s', ME.message);
        end
    end
end

fprintf('     - [CRS NOTICE] WKT is empty or invalid. Skipping VLR 2112 injection.\n');
end