% Esegui la simulazione a pacchetti sulla tua nuvola di punti stradale
logReport = run_SimulatedStreamingPipeline(ptCloudRoadOnly, 15.0, 0.1);

% Visualizza la tabella completa dei risultati
disp(logReport);

% Esempio di analisi post-streaming: Trova i pacchetti che hanno generato WARNING
warningIdx = find(contains(string(logReport.Var8), 'WARNING') | contains(string(logReport.Var8), 'FAIL'));
if ~isempty(warningIdx)
    fprintf('\nAttenzione! Rilevati %d pacchetti critici lungo il tracciato:\n', length(warningIdx));
    disp(logReport(warningIdx, {'Var1', 'Var5', 'Var6', 'Var8'}));
else
    fprintf('\nTutti i pacchetti sono rientrati nei parametri di sicurezza!\n');
end