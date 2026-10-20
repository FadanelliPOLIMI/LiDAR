function [GIS, Zterrain] = step7_prepareTerrainReport(ascFile, nodata, cellSize)
Zterrain = readmatrix(ascFile, 'FileType', 'text', 'NumHeaderLines', 6);
Zterrain = flipud(Zterrain);

maskNoData = (Zterrain == nodata);
if sum(maskNoData(:)) > 0
    Zterrain(maskNoData) = NaN;
    [nRows, nCols] = size(Zterrain);
    [Xg, Yg] = meshgrid(1:nCols, 1:nRows);
    maskValid = isfinite(Zterrain);
    F = scatteredInterpolant(Xg(maskValid), Yg(maskValid), Zterrain(maskValid), 'natural', 'nearest');
    Zterrain(maskNoData) = F(Xg(maskNoData), Yg(maskNoData));
end

samplerate = 1 / cellSize;
maxTrackCells = min(size(Zterrain,1), size(Zterrain,2)) - 1;
demStep = 1 / samplerate;
defaultTrackCells = max(1, min(round(2.0 * samplerate), maxTrackCells));
defaultTrackWidth = defaultTrackCells * demStep;

while true
    twText = strtrim(input(sprintf('TrackWidth [%.3f m; Invio = default] > ', defaultTrackWidth), 's'));
    if isempty(twText), trackWidthReq = defaultTrackWidth;
    else, trackWidthReq = str2double(strrep(twText, ',', '.')); end

    if ~isscalar(trackWidthReq) || trackWidthReq <= 0, continue; end
    trackCells = round(trackWidthReq * samplerate);
    if trackCells >= 1 && trackCells <= maxTrackCells
        trackWidth = trackCells * demStep;
        break;
    end
end

GIS.mode = 'real';
GIS.units = 'm';
GIS.Zmatrix = Zterrain;
GIS.TrackWidth = trackWidth;
GIS.samplerate = samplerate;
GIS.CutOff = 10;
GIS.Fext = {};
end