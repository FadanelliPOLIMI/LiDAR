
Lx = 1000;
sr = 2;
m = sr * Lx;
n = sr * Lx;
UCon = 12;
nFFT = 512;
ovlp = nFFT/4;
output = zeros(1,5);


loop_cnt = 1;
% related to exponent
for H = 0: 0.05: 1
    % inches RMS
    for sigma = 0.25: 0.05: 5
        % WNS cutoff, related to qr
        for CutOff = 50: 5: 150

            qr = 2*pi/CutOff;
            [ Z , ~, ~ ] = artificial_surf( sigma/UCon, H, Lx, m, n, qr );

            %  Calculate PSD of (current) synth surface
            [ psd, w_l ]  = pwelch( Z, nFFT, ovlp, nFFT, sr );

            % Discard first and last points
            w_l = w_l( 2:(end-1) );
            psd = psd( 2:(end-1), : );
            psd = mean( psd, 2 );

            %  Get Power Fit Coefficients
            [ pow1, int1 ] = Power_Fit_Coeffs( w_l, psd, CutOff );

            %  true RMS of surface
            RMSd = UCon * rms( Z, 'all' );

            % Midpoint Rule integration (rectangular)
            RMSfc = sqrt( int1/(pow1+1) * ((sr/2)^(pow1+1) - (1/CutOff)^(pow1+1) ) ) * UCon;

            output( loop_cnt, : ) = [ sigma, RMSfc, pow1, CutOff, H ]; %, qr, int1, RMSd,  ];

            loop_cnt = loop_cnt + 1;
        end
        fprintf( 'Current H = %f  |  Current sigma = %f\n', H, sigma )
    end
end
disp( 'done' );

save( 'SupportingCode\NewMat.mat', 'output' )


% SSmat = load( 'SupportingCode\SSmat.mat', 'output' );
% SSmat = SSmat.output;
% % SSmat = [ 1 sigma, 2 H, 3 CutOff, 4 qr, 5 pow1, 6 int1, 7 RMSd, 8 RMSfc ];
% H = interp3( SSmat(:,8), SSmat(:,5), SSmat(:,3), SSmat(:,2), sigma_fc, WNS_exp, CutOff )

%% Create Stats report of the output matrix

% Series of Crossplots
% 2D histograms etc




%%  Code to randomly generate paremeters to test the final surface

sr = 4;
Lx = 1000;
m = Lx*sr;
n = m;
nFFT = 1024;
ovlp = nFFT/4;
UCon = 12;

SSmat = load( 'NewMat.mat' );
SSmat = SSmat.output;

count = 1;

while count < 250

    WNS_exp = (rand()-0.5)*1.8 -2.1;%  scale to -1.2 to -3
    sigma_fc = (rand()-0.5)*3.75 + 2.125;%  scale to 0.25 to 4
    CutOff = (rand()-0.5)*75 + 87.5;%  scale to 50 to 125
    CutOff = round(CutOff/5)*5;  % round to even 5's

    % NewMat = [ 1 sigma (in), 2 RMSfc (in), 3 pow1, 4 CutOff (ft), 5 H ];
    ind = (SSmat(:,2)>(0.99*sigma_fc*UCon)) & (SSmat(:,2)<(1.01*sigma_fc*UCon));
    Fmat = SSmat( ind,: );  % Sigma_fc filter
    ind = (Fmat(:,3)>(WNS_exp-0.05)) & (Fmat(:,3)<(WNS_exp+0.05) );
    Fmat = Fmat( ind,: );   % pow1 filter
    ind = (Fmat(:,4)>(0.9*CutOff)) & (Fmat(:,4)<(1.1*CutOff));
    Fmat = Fmat( ind,: );   % CutOff filter
    
    qr = 2*pi/CutOff;
    H = mean( Fmat(:,5 ) );
    sigma = mean( Fmat(:,1) )/UCon;

    if numel( Fmat ) == 0
        % save bad parameters
        CheckMat(count,:) = [WNS_exp, sigma_fc, CutOff, H, sigma, 0 ];
    else
        % save good parameters
        [ Z , ~, ~ ] = artificial_surf( sigma, H, Lx, m, n, qr );
        
        % wdow = hamming( nFFT );
        [ psd_r, w_l ] = pwelch( Z', nFFT, ovlp, nFFT, sr );
        [ psd_c, ~ ]   = pwelch( Z,  nFFT, ovlp, nFFT, sr );
        
        % Discard first and last points
        w_l = w_l( 2:(end-1) );
        psd_r = psd_r( 2:(end-1), : );
        psd_c = psd_c( 2:(end-1), : );
                
        psd_m1 = mean( [ psd_r, psd_c], 2 );
        %  X/Y Power Fit Coefficients
        [ pow1, int1 ] = Power_Fit_Coeffs( w_l, psd_m1, CutOff );

        CheckMat(count,:) = [WNS_exp, sigma_fc, CutOff, H, sigma, pow1 ];
    end
    count = count + 1;
    if rem(count,10)==0
        fprintf( 'Status %d Loop\n', count )
    end
end

