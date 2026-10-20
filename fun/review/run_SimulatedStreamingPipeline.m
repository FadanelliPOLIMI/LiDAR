function streamLog = run_SimulatedStreamingPipeline(ptCloudRoad, chunkLength, resolution)
% RUN_SIMULATEDSTREAMINGPIPELINE - Simula lo streaming da file statico
% Taglia la nuvola in pacchetti spaziali e monitora RMSE, pendenze e incertezza al volo.

    if nargin < 2, chunkLength = 15.0; end % Lunghezza del pacchetto in metri lungo la strada
    if nargin < 3, resolution = 0.1; end   % Risoluzione griglia (10 cm)

    fprintf('=== AVVIO SIMULAZIONE STREAMING A PACCHETTI (Chunk: %.1fm) ===\n', chunkLength);

    xyz = double(ptCloudRoad.Location);
    x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

    % 1. INDIVIDUAZIONE DELL'ASSE LONGITUDINALE (PCA) per tagliare la strada in sequenza
    [~, score, ~] = pca([x, y]);
    s = score(:,1); % Coordinata di stazione longitudinale

    sMin = min(s);
    sMax = max(s);
    sEdges = sMin:chunkLength:sMax;

    streamLog = table();

    % 2. CICLO DI ELABORAZIONE A PACCHETTI (Simulazione Streaming)
    for i = 1:(length(sEdges)-1)
        sStart = sEdges(i);
        sEnd = sEdges(i+1);

        % Estrazione dei punti appartenenti al pacchetto corrente
        packetMask = (s >= sStart) & (s < sEnd);
        if sum(packetMask) < 30
            continue; % Salta pacchetti troppo scarsi
        end

        x_pk = x(packetMask);
        y_pk = y(packetMask);
        z_pk = z(packetMask);

        % Elaborazione del pacchetto (Interpolazione locale)
        try
            xGrid = min(x_pk):resolution:max(x_pk);
            yGrid = min(y_pk):resolution:max(y_pk);
            [Xq, Yq] = meshgrid(xGrid, yGrid);

            % Controllo di sicurezza per evitare griglie enormi nei singoli chunk
            if numel(Xq) > 50000
                xGrid = min(x_pk):(resolution*2):max(x_pk);
                yGrid = min(y_pk):(resolution*2):max(y_pk);
                [Xq, Yq] = meshgrid(xGrid, yGrid);
            end

            % Interpolatore locale robusto
            F = scatteredInterpolant(x_pk, y_pk, z_pk, 'natural', 'linear');
            Z_road = F(Xq, Yq);

            % Riempimento buchi ai bordi del pacchetto
            if any(isnan(Z_road(:)))
                F_near = scatteredInterpolant(x_pk, y_pk, z_pk, 'nearest', 'nearest');
                nanMask = isnan(Z_road);
                Z_road(nanMask) = F_near(Xq(nanMask), Yq(nanMask));
            end

            failed = false;
        catch
            failed = true;
        end

        % 3. CALCOLO DELLE METRICHE DI CONTROLLO E VALIDAZIONE
        if failed
            rmse = 999; maxSlope = 999; uncertainty = 999;
            status = {'FAIL (Crash Algoritmico)'};
        else
            % A. RMSE (Fedeltà ai punti originali del pacchetto)
            z_est = interp2(Xq, Yq, Z_road, x_pk, y_pk, 'linear');
            validZ = ~isnan(z_est);
            rmse = sqrt(mean((z_pk(validZ) - z_est(validZ)).^2));

            % B. Pendenza Massima (Stabilità per MSC ADAMS)
            [dz_dx, dz_dy] = gradient(Z_road, resolution, resolution);
            maxSlope = max([abs(dz_dx(:)); abs(dz_dy(:))]);

            % C. Incertezza locale (Proxy basato sulla dispersione dei dati nel pacchetto)
            uncertainty = std(z_pk);

            % D. COUNTER-ACTION / CONTROLLO DI SOGLIA
            if rmse > 0.05
                status = {'WARNING: Errore geometrico alto (Possibile cuneetta persa)'};
            elseif maxSlope > 1.0
                status = {'WARNING: Pendenza > 45° (Rischio shock in ADAMS)'};
            else
                status = {'OK (Pacchetto stabile)'};
            end
        end

        % 4. REGISTRAZIONE NEL LOG DI MONITORAGGIO
        newRow = {i, sStart, sEnd, sum(packetMask), rmse, maxSlope, uncertainty, status};
        streamLog = [streamLog; newRow];

        % Output in tempo reale sulla console di MATLAB
        fprintf('Chunk %02d [%.1fm - %.1fm] | Punti: %4d | RMSE: %.4f m | MaxSlope: %.2f rad | Esito: %s\n', ...
            i, sStart, sEnd, sum(packetMask), rmse, maxSlope, status{1});
    end

    fprintf('=== SIMULAZIONE STREAMING COMPLETATA CON SUCCESSO ===\n');
end