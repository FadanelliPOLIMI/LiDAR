function step8_launchTerrainReport(GIS)
risposta = questdlg('Vuoi procedere con TerrainReport?', 'Conferma', 'Sì', 'No', 'Sì');
if strcmp(risposta, 'Sì')
    figs = findall(groot,'Type','figure');
    for k = 1:numel(figs)
        try, figs(k).WindowState = 'minimized'; catch, end
    end
    TerrainReport_rectangular(GIS);
end
end