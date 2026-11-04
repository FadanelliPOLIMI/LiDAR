function TerrainReport_rectangular( varargin )
%  
%  This program will either accept GIS data passed into varargin, or create
%  a synthetic surface.  
%  The data must be 3D (a matrix of Z values), not 2D (will not work on a vector).
%  It's function is to create a Terrain Roughness Report which summarizes 
%  characteristics of the 3D surface.  There are also functions to export
%  the data to useful formats:  MAT, CSV, GeoTIFF, LAS, RDF, CRG, and/or PLY
%
%  A typical area is 1000x1000 ft at 4 samples per foot
%  A typical area is 305x305 m at 10 samples per meter
%  Larger areas are possible, limitted by RAM
%  Smaller areas may generate unusual frequency domain output.
%  
%  Three modes to run:  
%  1)  No input data (generate a synthetic surface with default parameters)
%  2)  Call with struct, specific parameters to create a synthetic surface
%  3)  Call with struct, external GIS data are given for analysis
%
%  OPTION 1 - varargin is empty
%  Default values used to create a synthetic surface
%
%  OPTION 2 - varargin is a struct with the following fields:
%  1)  varargin.mode   = 'synth'
%  2)  varargin.units  = ('ft' or 'm'), linear units of feet or meters
%  3)  varargin.length = (number), X/Y length in above units
%  4)  varargin.samplerate = (number), samples per linear unit
%  5)  varargin.RMS    = (number), design output RMS [in/cm], CutOff limitted
%  6)  varargin.EXP    = (number), design output exponent of power fit (aka waviness)
%  7)  varargin.CutOff = (number), wavelength upper cutoff in linear units above
%  8)  varargin.TrackWidth = (number), vehicle track width for coherence in above units
%  
%  OPTION 3 - varargin is a struct with the following fields:
%  1)  varargin.mode    = 'real'
%  2)  varargin.units   = ('ft' or 'm'), linear units of feet or meters
%  3)  varargin.Zmatrix = matrix of Z data, in above units
%  4)  varargin.TrackWidth = (number), vehicle track width for coherence in above units
%  5)  varargin.samplerate = (number), samples per linear unit
%  6)  varargin.CutOff     = (number), WNS wavelength upper cutoff in linear units above
%  
%  % Example Opt 1:
%  % This will create a synth surf with default values
%  TerrainReport;     % Or just hit "Run"
%
%  % Example Opt 2:
%  % Design a surface and call pragmatically
%  varargin.mode   = 'synth';
%  varargin.units  = 'ft';
%  varargin.length = 1000; 
%  varargin.samplerate = 4; 
%  varargin.RMS    = 2.5;  % inches RMS
%  varargin.EXP    = -2.0;
%  varargin.CutOff = 100; 
%  varargin.TrackWidth = 7.5;
%  TerrainReport( varargin ) 
%
%  % Example Opt 3:
%  % Analyize 3D data from a different source
%  varargin.mode    = 'real';
%  varargin.units   = 'm';
%  varargin.Zmatrix = ZDATA;  % matrix of Z-data in workspace
%  varargin.TrackWidth = 2;   % meter width
%  varargin.samplerate = 10;  % samples per meter
%  varargin.CutOff     = 20;  % meter/cy WNS cutoff
%  TerrainReport( varargin );
%
%  Based on:
%  "Quantitative characterization of surface topography using spectral analysis"
%  Tevis Jacobs, Till Junge, Lars Pastewka
%  Journal reference:	Surface Topography: Metrology and Properties 5 (1), 013001 (2017)
%  https://www.mathworks.com/matlabcentral/fileexchange/60817-surface-generator-artificial-randomly-rough-surfaces
%  
%  Supporting code:
%  Requires Signal Processing Toolbox 
%       PWELCH for the Power Spectal Density (Wave Number Spectrum)
%       MSCOHERE for the track coherence
%  OpenCRG       https://www.asam.net/standards/detail/opencrg/
%  NaN Errors    https://www.mathworks.com/matlabcentral/fileexchange/6837-nan-suite
%  GeoTIFF files https://www.mathworks.com/matlabcentral/fileexchange/27959-geotiffwrite
%  LAS files     https://www.mathworks.com/matlabcentral/fileexchange/66033-lasio
%  PLY files     https://www.mathworks.com/matlabcentral/fileexchange/55171-plywrite
%  2D Filter     https://www.mathworks.com/matlabcentral/fileexchange/46189-generating-gaussian-filter-2d-matrix
%  
%  Information passed to ARTIFICIAL_SURF.M
%
%  [z , PixelWidth , PSD] = artificial_surf(sigma, H, Lx, m , n, qr)
%
% ======================= inputs
% parameters (in SI units)
% sigma: standard deviation , i.e. root-mean-square roughness Rq(m)
% H: Hurst exponent (roughness exponent), 0<= H <= 1
% It relates to the fractal dimension of a surface by D = 3-H.
% Lx: length of topography in x direction.
% m: number of pixels in x
% n: number of pixels in y
%
% !!! please note that setting x and y to be a power of 2 will reduce
% computing time, like using the numbers (256,512,1024,2048,...)
% !!! a normal desktop computer cannot handle pixels above 2048
% !!! you can increase resolution (i.e. pixelwidth) by increasing n and m,
% or reducing size of final topography (reducing Lx)
%
% qr (((OPTIONAL parameter)))):
% cut-off wavevector or roll-off wavevector (1/m), it relates to
% roll-off wavelength by qr = (2*pi)/lambda_r, where lambda_r is the
% roll-off wavelength. Check the image that I uploaded to Mathworks for its
% meaning.
% !!! note that qr cannot be smaller than image size, i.e. qr > (2*pi/L)
% where L is the length of image in x and/or y direction ( Lx and Ly ).
% Also, qr cannot be bigger than Nyquist frequency qr < (pi/PixelWidth)
%
% ======================== outputs
% z : the surface height profile of a randomly rough surface (m)
% PixelWidth : spatial resolution, i.e. the distance between each two points
% in the generated height topography z (m). This directly relates to the size
% of generated surface by the relation Lx = (m-1) * PixelWidth
% PSD: this is a radially averaged power spectrum  of the generated surface.

%%  Initialize program

%  Add SupportingCode folder to path  =============================
% Get the actual folder where this script is saved (more robust than pwd)
[folder, ~, ~] = fileparts(mfilename('fullpath'));
if isempty(folder)
    folder = pwd; % Fallback if run as an unsaved script/command
end
targetPath = fullfile(folder, 'SupportingCode');

% Check if the SupportingCode folder is already on the MATLAB path
pathCell = regexp(path, pathsep, 'split');
if ispc  % Windows is not case-sensitive
    onPath = any(strcmpi(targetPath, pathCell));
else     % Mac/Linux are case-sensitive
    onPath = any(strcmp(targetPath, pathCell)); % FIXED comparison logic
end

if ~onPath
    addpath(targetPath);
end

%  check for Gaussian_filter code
if ~exist( 'Gaussian_filter.m', 'file'  )
    disp( 'Gaussian Filter subroutine not found.  Please download here:')
    url = 'https://www.mathworks.com/matlabcentral/fileexchange/46189-generating-gaussian-filter-2d-matrix';
    fprintf( 1,'<a href="matlab: web(''%s'') ">%s</a>\n', url, url );
    disp( 'Program initialization failed :(' )
    return
end

%  check for NAN-suite code
if ~exist( 'nanmean.m', 'file'  )
    disp( 'NANMEAN subroutine not found.  Please download here:')
    url = 'https://www.mathworks.com/matlabcentral/fileexchange/6837-nan-suite';
    fprintf( 1,'<a href="matlab: web(''%s'') ">%s</a>\n', url, url );
    disp( 'Program initialization failed :(' )
    return
end


if ~isempty( varargin )
    % convert cell to struct
    varargin = varargin{1};
else
    %  varargin does not exist
    %  define default design parameters
    varargin.mode   = 'synth';
    varargin.units  = 'm';
    varargin.length = 1000;
    varargin.samplerate = 4;
    varargin.RMS    = 2.5;  % inches RMS
    varargin.EXP    = -2.0;
    varargin.CutOff = 100;
    varargin.TrackWidth = 6.75;  % MTVR ~ 6.75 ft
    varargin.Fext = {};  % Fext optional
end

%  varargin is defined for real data
if  strcmpi( varargin.mode, 'real' )

    disp( 'Surface has been defined')
    disp( 'Terrain Roughness report will be generated')

units = varargin.units;

Zabs = varargin.Zmatrix;       % DEM ORIGINALE con quote reali
Z    = Zabs - mean(Zabs,'all'); % DEM centrato SOLO per le analisi
wkt  = varargin.wkt;
TW    = varargin.TrackWidth;
sr    = varargin.samplerate;
CutOff = varargin.CutOff;

PixelWidth = 1/sr;

    if isfield( varargin, 'Fext' )
        Fext = varargin.Fext;
        % Fext is optional
    else
        Fext = {};
    end

    [ m, n ] = size( Z );
    Lx = n / sr;
    Ly = m / sr;
    pts = m*n;

    switch units
        case 'ft'
            UCon = 12;
            Urms = 'in';
            % Keep the user-supplied CutOff.
            nFFT = 2^nextpow2( sr / 0.004 + 1 );
        case 'm'
            UCon = 100;
            Urms = 'cm';
            % Keep the user-supplied CutOff.
            nFFT = 2^nextpow2( sr / 0.01 + 1 );
    end

    sigma = rms( Z, 'all' );  %  * UCon;
    sigma_fc = sigma;

    %  DEFINE FFT BLOCK SIZE AND GRID PARAMETERS
    % nFFT = 1024;    %  block size for PSD and Coherence
    overlap = 25;    % percent overlap for RMS grid and PDF stationarity
    
    disp(' ')
    
    PSname = sprintf('RSurf_Dim%.0fx%.0f%s_Res%0.2f%s_Wc%.0f%s_V', ...
                        Lx, Ly, units, 1/sr*UCon, Urms, CutOff, units);
    PSname = replace( PSname, '.', 'p' );
    
    sur = 0;
    while exist( sprintf( '%s%02.0f.pdf', PSname, sur ), 'file' )
        sur = sur+1;
    end
    PSname = sprintf( '%s%02.0f', PSname, sur);
    fprintf( 'Output file(s) rootname:\n' )
    fprintf( '%s\n', PSname )
    
    Froot = sprintf( 'Input Surface: Area= %.0fx%.0f %s, %0.2f %s resolution, Wc: %.0f %s/cy', ...
                        Lx, Ly, units, sr, Urms, CutOff, units );


elseif strcmpi( varargin.mode, 'synth' )

    %  check for artificial_surface code
    if ~exist( 'artificial_surf.m', 'file'  )
        disp( 'Artificial Surface subroutine not found.  Please download here:')
        url = 'https://www.mathworks.com/matlabcentral/fileexchange/60817-surface-generator-artificial-randomly-rough-surfaces';
        fprintf( 1,'<a href="matlab: web(''%s'') ">%s</a>\n', url, url );
        disp( 'Program initialization failed :(' )
        return
    end

    % save parameters to workspace variables
    units = varargin.units;
    sigma = varargin.RMS;
    Lx = varargin.length;
    sr = varargin.samplerate;    
    WNS_exp = varargin.EXP;
    CutOff = varargin.CutOff;
    TW = varargin.TrackWidth; 
    Ly = Lx;
    % percent overlap for RMS grid and PDF stationarity
    overlap = 25;

    if isfield( varargin, 'Fext' )
        Fext = varargin.Fext;
        % Fext is optional
    else
        Fext = {};
    end
    
    %  DEFINE FFT BLOCK SIZE AND GRID PARAMETERS
    %  block size for PSD and Coherence.  Use power of 2
    switch units
        case 'ft'
            %  base of 0.0039 cy / ft
            nFFT = 2^nextpow2( sr / 0.004 + 1 );
        case 'm'
            %  base of 0.01 cy / m
            nFFT = 2^nextpow2( sr / 0.01 + 1 );
    end

    %  if surface is too small, must step nFFT down
    while nFFT > ( sr * Lx )
        nFFT = nFFT / 2;
    end

    % m: number of pixels in x/y
    m = Lx * sr;        % m/ft, length * samples per unit
    n = Ly * sr;        % feet - length * samples per unit
    pts = m*n;
    
    % set minor units and conversion factors
    switch units
        case 'm'
            UCon = 100; % m to cm
            Urms = 'cm';
        case 'ft'
            UCon = 12; % ft to inches
            Urms = 'in';
    end
    
    if CutOff > (Lx/2) || (exist('CutOff', 'var')==0)
    %  Using no CutOff is not recommended.
    %  This produces unusual results with all output tests.
    %  Functionally CutOff is set to 9999 for external surface data, but
    %  limited to one point before the minimum wavenumber for synthetic

        % one unit less than the theoretical max wavelength
        CutOff = nFFT/sr - 1;  
        % set PDF/CDF block size
        switch units
            case 'm'
                Swin = 10;
            case 'ft'
                Swin = 32;
        end
    else
        %  RMS grid and PDF overlap
        Swin = CutOff;
    end
    
    %  convert from minor unit to major unit
    sigma = sigma / UCon;

    % CutOff RMS
    sigma_fc = sigma;

    % Output Filename
    %  Synth_H0p5_RMS3p00_128Sq_10w.las
    % PSname = sprintf( 'SSurf_Exp%2.1f_RMS%2.1f%s_Dim%.0fX%.0f%s_Wc%.0f%s_V', ...
    %                     WNS_exp, sigma_fc*UCon, Urms, Lx, Ly, units, CutOff, units );
    PSname = sprintf( 'SSurf_Exp%2.2f_RMS%2.2f%s_Wc%.0f%s_V', ...
                        WNS_exp, sigma_fc*UCon, Urms, CutOff, units );

    PSname = replace( PSname, '.', 'p' );
    
    sur = 0;
    while exist( sprintf( 'AVT_Out\\%s%02.0f.pdf', PSname, sur ), 'file' )
        sur = sur+1;
    end
    PSname = sprintf( '%s%02.0f', PSname, sur);

    disp( ' ' )
    disp( 'Synthetic Terrain inputs:' )
    fprintf( '1)  UNITS, Linear units of feet or meters = %s and %s\n', units, Urms )
    fprintf( '2)  Lx=Ly, Dimensions in above units  \t  = %.0f %s\n', Lx, units )
    fprintf( '3)  SAMPLERATE in both directions     \t  = %.0f samples per %s\n', sr, units )
    fprintf( '4)  CUTOFF, wavelength upper cutoff (Wc)  = %.0f %s / cycle\n', CutOff, units )
    % fprintf( '5)  RMS, design Full RMS roughness    \t  = %.2f %s\n', sigma*UCon, Urms )
    % if CutOff < 9999
        fprintf( '5)  %.0f %s RMS, Limited RMS roughness \t  = %.2f %s\n', ...
            CutOff, units, sigma_fc*UCon, Urms ); 
    % end
    fprintf( '6)  EXP, design exponent (aka waviness)\t  = %.2f\n', WNS_exp )
    fprintf( '7)  TRACK, track width for coherence  \t  = %2.1f %s\n', TW, units)
    % fprintf( '9)  FEXT, output file format(s) desired\t  = %s\n', ...
    %     sprintf( '%s, ', Fext{:} ) )
    fprintf( 'Output file(s) rootname:\n' )
    fprintf( '%s\n', PSname )
    Froot = sprintf( 'Synthetic Surface: Exp: %2.1f, RMS: %2.2f %s, Area: %.0fx%.0f %s, Wc: %.0f %s/cy', ...
                        WNS_exp, sigma_fc*UCon, Urms, Lx, Ly, units, CutOff, units );
    disp( ' ' )

    str = input( 'Press ENTER to continue with these parameters or [N/n] to exit > ', 's' );
    % str = input( 'Continue with these parameters [YES] or EXIT program to revise? [y/n]  > ', 's' );
    if contains( lower(str), 'n' )
        return
    end
    disp( ' ' )

    %  Calculate H to fit the expontent and RMS values  ==================
    % H: Hurst exponent (roughness exponent), 0 <= H <= 1
    % H relates to the fractal dimension of a surface by D = 3-H.

    SSmat = load( 'NewMat.mat' );
    SSmat = SSmat.output;
    
    if strcmp( units, 'm' )
        SSmat(:,1) = SSmat(:,1) *2.54;  % sigma in to cm
        SSmat(:,2) = SSmat(:,2) *2.54;  % RMSfc in to cm
        SSmat(:,4) = SSmat(:,4) *12*2.54/100;  % CutOff ft to m
    end

    qr = 2*pi/CutOff;

    % NewMat = [ 1 sigma (in), 2 RMSfc (in), 3 pow1, 4 CutOff (ft), 5 H ];
    ind = (SSmat(:,2)>(0.99*sigma_fc*UCon)) & (SSmat(:,2)<(1.01*sigma_fc*UCon));
    Fmat = SSmat( ind,: );  % Sigma_fc filter
    ind = (Fmat(:,3)>(WNS_exp-0.05)) & (Fmat(:,3)<(WNS_exp+0.05) );
    Fmat = Fmat( ind,: );   % pow1 filter
    ind = (Fmat(:,4)>(0.9*CutOff)) & (Fmat(:,4)<(1.1*CutOff));
    Fmat = Fmat( ind,: );   % CutOff filter

    if numel( Fmat ) == 0
        disp( 'The parameters entered are outside of the design space.')
        disp( 'Please review this information to understand the problem.')
        disp( 'TerrainReport execution ended')
        winopen( 'SupportingCode\MatrixVisual.pdf' )
        return
    else
        % matrix is in ft
        H = mean( Fmat(:,5 ) );
        sigma = mean( Fmat(:,1) )/UCon;
        [ Z , PixelWidth, ~ ] = artificial_surf( sigma, H, Lx, m, n, qr );
    end
    
    clear Hlx Hm Hn xv yv ind pow1 int1 wavy cnt psd w_l coeffs1 CC ind ...
        Hloop_cnt correction C flat hi Hst i1 lo onPath qr str SSmat Fmat % Z
end

% Synthetic mode does not define Zabs in the original workflow.
% For preview purposes, use the generated surface itself.
if ~exist('Zabs','var')
    Zabs = Z;
end

%% =========================================================
% ADATTAMENTO AUTOMATICO nFFT PER DEM RETTANGOLARI
% X, Y e diagonali vengono trattate separatamente.
% Richiede almeno 4 segmenti Welch in ciascuna direzione.
% ==========================================================

[mDEM, nDEM] = size(Z);       % mDEM = righe (Y), nDEM = colonne (X)

if overlap > 1
    overlapFrac = overlap / 100;
else
    overlapFrac = overlap;
end

minSegments = 4;
nFFT_base = nFFT;

% ----- X: profili lungo le colonne, lunghezza = nDEM -----
nFFT_X = min(nFFT_base, 2^floor(log2(nDEM)));
nSegments_X = 0;

while nFFT_X >= 8
    ovlp_X_test = round(overlapFrac * nFFT_X);
    step_X_test = nFFT_X - ovlp_X_test;
    nSegments_X = 1 + floor((nDEM - nFFT_X) / step_X_test);
    if nSegments_X >= minSegments
        break;
    end
    nFFT_X = nFFT_X / 2;
end

% ----- Y: profili lungo le righe, lunghezza = mDEM -----
nFFT_Y = min(nFFT_base, 2^floor(log2(mDEM)));
nSegments_Y = 0;

while nFFT_Y >= 8
    ovlp_Y_test = round(overlapFrac * nFFT_Y);
    step_Y_test = nFFT_Y - ovlp_Y_test;
    nSegments_Y = 1 + floor((mDEM - nFFT_Y) / step_Y_test);
    if nSegments_Y >= minSegments
        break;
    end
    nFFT_Y = nFFT_Y / 2;
end

% ----- Diagonali: lunghezza massima = min(mDEM,nDEM) -----
diagSignalLength = min(mDEM, nDEM);
nFFT_D = min(nFFT_base, 2^floor(log2(diagSignalLength)));
nSegments_D = 0;

while nFFT_D >= 8
    ovlp_D_test = round(overlapFrac * nFFT_D);
    step_D_test = nFFT_D - ovlp_D_test;
    nSegments_D = 1 + floor((diagSignalLength - nFFT_D) / step_D_test);
    if nSegments_D >= minSegments
        break;
    end
    nFFT_D = nFFT_D / 2;
end

if nFFT_X < 8 || nFFT_Y < 8 || nFFT_D < 8
    error(['Area selezionata troppo piccola per una stima spettrale ' ...
           'affidabile in tutte le direzioni.']);
end

fprintf('\n');
fprintf('============================================\n');
fprintf('ADATTAMENTO ANALISI SPETTRALE RETTANGOLARE\n');
fprintf('============================================\n');
fprintf('Dimensione DEM : %d righe (Y) x %d colonne (X)\n', mDEM, nDEM);
fprintf('nFFT base      : %d\n', nFFT_base);
fprintf('nFFT X         : %d  (%d segmenti Welch)\n', nFFT_X, nSegments_X);
fprintf('nFFT Y         : %d  (%d segmenti Welch)\n', nFFT_Y, nSegments_Y);
fprintf('nFFT diagonali : %d  (%d segmenti Welch sul lato corto)\n', nFFT_D, nSegments_D);
fprintf('Overlap        : %.0f %%\n', overlapFrac * 100);
fprintf('============================================\n');

% Da qui in poi overlap viene usato come frazione 0...1
overlap = overlapFrac;

% Compatibilita' con eventuali parti legacy
nFFT = min(nFFT_X, nFFT_Y);


%  Check available RAM
disp(' ')
%memory
% switch canUseGPU
%     case 0
%         disp( 'There is not a GPU available for calculation and/or rendering' )
%     case 1
%         disp( 'There is a GPU available, however the code is not optimized' )
% end
disp( 'MATLAB can render/print the surface, however GIS and Pointcloud' )
disp( 'software is better suited for this task.')
disp(' ')



%% START OF SURFACE GRAPH (OPTIONAL)

%  Create 3D plot of the surface, and then prints figure 4 different views

str = input( 'Press ENTER to continue, or [P/p] to Print 3-D Surface Rendering > ', 's' );
% str = input( 'Print 3-D Surface Rendering? [y/n]  > ', 's' );
disp(' ')
if numel( str ) > 0
    if strcmpi( str(1), 'p' ) == 1

        % Create and Format Figure
        f1 = figure( 'Theme', 'light', 'MenuBar', 'none', 'ToolBar', 'none' );
        f1.PaperOrientation = 'landscape';
        f1.WindowState   = 'normal';
        f1.Units      = 'inches';
        f1.Position   = [ 0 0 11 8.5 ];
        f1.PaperUnits = 'inches';
        f1.PaperSize  = [ 11 8.5 ];
        f1.PaperPosition = [ 0 0 11 8.5 ];
        movegui('center')
        pause( 0.5 );

        %  create X/Y matrices - for plot and export
        % Z e' m x n: m righe = Y, n colonne = X
        x = (0:n-1) * PixelWidth;
        y = (0:m-1) * PixelWidth;
        [X,Y] = meshgrid(x,y);

        surf( X, Y, Z*UCon );

        % graph_format( gca, Lx )
        xlabel( sprintf( 'X-Longitude (%s)', units ) )
        ylabel( sprintf( 'Y-Latitude (%s)', units ) )
        zlabel( sprintf( 'Z-Elevation (%s)', Urms ) )
        shading interp;  grid on;  box on;  camlight;
        xlim( [ 0 Lx ] )
        ylim( [ 0 Ly ] )

        %  Create folder to save output GIS data
        if ~exist( [ pwd, '\AVT_Out\' ], 'dir' )
            mkdir( [ pwd, '\AVT_Out\' ] );
        end

        %  loop through 4 views, printing one at a time
        for st = 1: 1: 4
            switch st
                case 1
                    fprintf( 'Printing Isometric View\n' );
                    view( 3 )
                    title( { Froot; 'Isometric View' } )
                    % PV = '_ISO';
                case 2
                    fprintf( 'Printing X-Z View\n' );
                    view( 0, 0 );
                    title( { Froot; 'X-Z View' } )
                    % PV = '_X-Z';
                case 3
                    fprintf( 'Printing Y-Z View\n' );
                    view( 90, 0 );
                    title( { Froot; 'Y-Z View' } )
                    % PV = '_Y-Z';
                case 4
                    fprintf( 'Printing X-Y View\n' );
                    view( 0, 90 );
                    title( { Froot; 'X-Y View' } )
                    % PV = '_X-Y';
            end
            % print( f1, PSname, '-depsc', '-fillpage', '-append' );
            % print( f1, [PSname PV], '-dpdf', '-fillpage' );
            exportgraphics( f1, ['AVT_Out\' PSname '.pdf'], 'Append', true, 'Padding', 'tight' );
        end

        % cleanup workspace for RAM
        clear x y
        % close( f1 )

        % GAUSSIAN weighting
        % Swin = 3;
        ConMat2 = Gaussian_filter( Swin*sr/2, Swin*sr*sigma );
        Zmu  = conv2( Z, ConMat2, 'same');       %  Smoothed surface
        Zdt = Z - Zmu;                           %  Detrended Surface

        % add smoothed surface to surface graph
        hold on
        surf( X, Y, Zmu*UCon, 'FaceColor', [0,0,0] );
        title( { Froot; ...
            sprintf('Isometric View - RAW and %.0f ft SMOOTHED Surface', Swin ) } )
        view( 3 )

        %  PRINT OFF OVERLAY
        % PV = '_over';
        fprintf( 'Printing Raw & Smoothed Surface\n' )
        % print( f1, PSname, '-dpsc', '-fillpage', '-append' );
        % print( f1, [PSname PV], '-dpdf', '-fillpage' );
        exportgraphics( f1, ['AVT_Out\' PSname '.pdf'], 'Append', true, 'Padding', 'tight' );

        cla;
        surf( X, Y, Zmu*UCon, 'EdgeColor', 'none' );
        xlabel( sprintf( 'X-Longitude (%s)', units ) )
        ylabel( sprintf( 'Y-Latitude (%s)', units ) )
        zlabel( sprintf( 'Z-Elevation (%s)', Urms ) )
        shading interp;  grid on;  box on;  camlight;
        xlim( [ 0 Lx ] )
        ylim( [ 0 Ly ] )
        title( { Froot; ...
            sprintf('Isometric View - %.0f %s SMOOTHED Surface', Swin, units ) } )
        view( 3 )

        fprintf( 'Printing Smoothed Surface\n' );

        % PV = '_sm';
        % print( f1, PSname, '-dpsc', '-fillpage', '-append' );
        % print( f1, [PSname PV], '-dpdf', '-fillpage' );
        exportgraphics( f1, ['AVT_Out\' PSname '.pdf'], 'Append', true, 'Padding', 'tight' );

        %  Plot DETRENDED surface by itself
        cla
        surf( X, Y, Zdt*UCon, 'EdgeColor', 'none' ); %, 'FaceColor', [0,0,0] )
        xlabel( sprintf( 'X-Longitude (%s)', units ) )
        ylabel( sprintf( 'Y-Latitude (%s)', units ) )
        zlabel( sprintf( 'Z-Elevation (%s)', Urms ) )
        shading interp;  grid on;  box on;  camlight;
        xlim( [ 0, Lx ] )
        ylim( [ 0, Ly ] )
        view( 3 )
        title( { Froot; ...
            sprintf('Isometric View - %.0f %s DETRENDED Surface', Swin, units ) } )

        % old print methods
        % print( f1, PSname, '-dpsc', '-fillpage', '-append' );
        % print( f1, [PSname PV], '-dpdf', '-fillpage' );
        exportgraphics( f1, ['AVT_Out\' PSname '.pdf'], 'Append', true, 'Padding', 'tight' );

        %  Close 3D Surface Figure
        close( f1 );
    end
end


%%  Print off input parameters
%
%                   NG-NRMM Natural 3-D Terrain Report
%               NG-NRMM Synthetic Terrain Generation Report
%

%  Create figure
% f2 = figure( 'Color', 'white' );
f2 = figure( 'Theme', 'light', ...
            'MenuBar', 'none', ...
            'ToolBar', 'none')
            %'WindowStyle','alwaysontop' );
f2.Units = 'inches';
f2.Position   = [ 0 0 8.5 11 ];
f2.WindowState   = 'normal';
f2.PaperOrientation = 'portrait';
f2.PaperUnits = 'inches';
f2.PaperSize  = [ 8.5 11 ];
f2.Position   = [ 0 0 8.5 11 ];
movegui('center')
pause( 0.5 );
subplot( 3, 2, 1 );

fprintf( 'Creating Terrain Report...\n' )

if strcmp( varargin.mode, 'synth' )
    %  This code is for the synthetic terrain
    hi = 1/CutOff;
    lo = sr/2;  
    C = ( WNS_exp+1 ) * ( sigma_fc^2 / (lo^(WNS_exp+1) - hi^(WNS_exp+1) ) );

    % Graphic output of input parameters? 
    sgtitle('NG-NRMM Synthetic Terrain Report', 'FontWeight', 'bold' )
    title('Input Parameters')
    
    STR_cell = { sprintf( 'Linear Units:  %s and %s', units, Urms )
                 sprintf( 'Dimensions:\t %d X %d %s', Lx, Ly, units )
                 sprintf( 'Resolution:\t  %d samples per %s', sr, units )
                 % sprintf( 'Treated as (square) AREA or (rectangular) LANE' )
                 sprintf( '' )
                 sprintf( 'Synthetic Terrain Generation parameters' )
                 sprintf( 'Upper wavelength cutoff (Wc) = %d %s/cy', CutOff, units )
                 sprintf( 'Design RMS = %0.2f %s (%0.2f %s)', ...
                    sigma_fc*UCon, Urms, sigma*UCon, Urms )
                 % sprintf( 'H: Hurst (roughness) exponent , 0 <= H <= 1')
                 % sprintf( 'H relates to the fractal dimension of a surface by D = 3H')
                 sprintf( 'Design WNS exponent ~ Hurst Exponent  ')
                 sprintf( 'WNS Exp = %0.2f  ~~>  H = %0.2f', WNS_exp, H )
                 sprintf( 'WNS Intercept ~= %0.2e ', C )
                 sprintf( '' )
                 sprintf( 'Output rootname:' )
                 sprintf( '%s', PSname )
                 };
else
    %  This code is for the real DEM input

    sgtitle('NG-NRMM Input Terrain Report', 'FontWeight', 'bold')
    title('Input Metadata')

    STR_cell = { sprintf( 'An input GIS file was given.' )
                 sprintf( 'Descriptive metadata is shown below:' )
                 sprintf( '' )
                 sprintf( 'Linear Units:  %s', units )
                 sprintf( 'Dimensions: %.1f X %.1f %s', Lx, Ly, units )
                 sprintf( 'Resolution: %0.1f samples per %s', sr, units )
                 sprintf( '' )
                 sprintf( 'Output rootname:' )
                 sprintf( '%s', PSname )
                 };
end

if numel( Fext ) > 0
% if exist( 'Fext', 'var' )
    %  print out list out output files requested
    if max(ismember( Fext, 'mat'));  STR_cell = [ STR_cell; sprintf( '%s.mat', PSname ) ]; end
    if max(ismember( Fext, 'las'));  STR_cell = [ STR_cell; sprintf( '%s.las', PSname ) ]; end
    if max(ismember( Fext, 'tif'));  STR_cell = [ STR_cell; sprintf( '%s.tif', PSname ) ]; end
    if max(ismember( Fext, 'csv'));  STR_cell = [ STR_cell; sprintf( '%s.csv', PSname ) ]; end
    if max(ismember( Fext, 'crg'));  STR_cell = [ STR_cell; sprintf( '%s.crg', PSname ) ]; end
    if max(ismember( Fext, 'rdf'));  STR_cell = [ STR_cell; sprintf( '%s.rdf', PSname ) ]; end
end


xlim( [ 0 1 ] )
ylim( [ 0 1 ] )
text( 0, 1, STR_cell, 'FontSize', 9,...
    'Interpreter', 'none', 'VerticalAlignment', 'top' )
axis off

%% =========================================================
% DEM PREVIEW - QUOTE REALI
% ==========================================================

subplot(3,2,2);

% Coordinate locali
xDEM = (0:n-1) / sr;
yDEM = (0:m-1) / sr;

% Visualizza il DEM ORIGINALE
imagesc(xDEM, yDEM, Zabs);

% Orientamento corretto
set(gca,'YDir','normal');

axis equal;
axis tight;

xlabel(sprintf('Longitude (%s)', units), 'Color','k');
ylabel(sprintf('Latitude (%s)', units), 'Color','k');

title(sprintf('DEM Preview, Z-units = %s', units), ...
      'Color','k');

colormap(parula);
colorbar('Color','k');

grid on;
box on;

drawnow;
%%  Standard 1-D PSD Calculation - RECTANGULAR SAFE

fprintf( 'Calculating Directional PSDs\n' )

% Z is m x n:
%   rows    -> Y coordinate
%   columns -> X coordinate
% pwelch works down matrix columns:
%   Z' -> X profiles (signal length n)
%   Z  -> Y profiles (signal length m)

ovlp_X = round(overlap * nFFT_X);
ovlp_Y = round(overlap * nFFT_Y);
ovlp_D = round(overlap * nFFT_D);

[psd_X, w_X] = pwelch(Z', nFFT_X, ovlp_X, nFFT_X, sr);
[psd_Y, w_Y] = pwelch(Z,  nFFT_Y, ovlp_Y, nFFT_Y, sr);

% Discard DC and final point as in the original code
w_X   = w_X(2:end-1);
w_Y   = w_Y(2:end-1);
psd_X = psd_X(2:end-1,:);
psd_Y = psd_Y(2:end-1,:);

% Common physical wavenumber grid for X/Y averaging and power fit
wMinXY = max(min(w_X), min(w_Y));
wMaxXY = min(max(w_X), max(w_Y));

if wMaxXY <= wMinXY
    error('No common X/Y wavenumber interval is available.');
end

nCommonXY = min(numel(w_X), numel(w_Y));
w_XY = linspace(wMinXY, wMaxXY, nCommonXY)';

psd_X_common = interp1(w_X, psd_X, w_XY, 'linear');
psd_Y_common = interp1(w_Y, psd_Y, w_XY, 'linear');

mean_psd_X = mean(psd_X_common, 2);
mean_psd_Y = mean(psd_Y_common, 2);

% Equal directional weighting, independent of rectangle aspect ratio
psd_m1 = (mean_psd_X + mean_psd_Y) / 2;


% ==========================================================
% DIAGONAL PSDs WITHOUT SPDIAGS OR ARTIFICIAL ZERO PADDING
% ==========================================================

dsr = sr / sqrt(2);  % samples per unit distance along a 45 deg diagonal

requiredDiagSamples = nFFT_D + ...
    (minSegments-1) * (nFFT_D - ovlp_D);

diagOffsets = -(m-1):(n-1);
D1cells = {};
D2cells = {};
validDiagOffsets = [];

psd_D1 = [];
psd_D2 = [];
w_D_full = [];

for kk = diagOffsets
    d1 = diag(Z, kk);
    d2 = diag(fliplr(Z), kk);

    if numel(d1) >= requiredDiagSamples && ...
       numel(d2) >= requiredDiagSamples

        [pD1, wtmp] = pwelch(d1, nFFT_D, ovlp_D, nFFT_D, dsr);
        [pD2, ~]    = pwelch(d2, nFFT_D, ovlp_D, nFFT_D, dsr);

        psd_D1(:,end+1) = pD1; %#ok<AGROW>
        psd_D2(:,end+1) = pD2; %#ok<AGROW>

        D1cells{end+1} = d1; %#ok<AGROW>
        D2cells{end+1} = d2; %#ok<AGROW>
        validDiagOffsets(end+1) = kk; %#ok<AGROW>
        w_D_full = wtmp;
    end
end

if isempty(psd_D1)
    error(['No diagonal contains enough real samples for the requested ' ...
           'Welch analysis. Enlarge the selected area.']);
end

w_D    = w_D_full(2:end-1);
psd_D1 = psd_D1(2:end-1,:);
psd_D2 = psd_D2(2:end-1,:);

mean_psd_D1 = mean(psd_D1,2);
mean_psd_D2 = mean(psd_D2,2);
psd_m2 = (mean_psd_D1 + mean_psd_D2) / 2;


% ==========================================================
% PSD graph
% ==========================================================

subplot(3,2,3);

allPSD_common = [psd_X_common, psd_Y_common];

p1 = loglog(w_XY, min(allPSD_common,[],2), 'Color', [.8 .8 .8]);
hold on; grid on; box on;

fill([w_XY; flipud(w_XY)], ...
     [min(allPSD_common,[],2); flipud(max(allPSD_common,[],2))], ...
     [.8 .8 .8], 'EdgeColor', [.8 .8 .8]);

p2 = loglog(w_X, mean(psd_X,2), '-',  'Color', 'b');
p3 = loglog(w_Y, mean(psd_Y,2), '--', 'Color', 'g');
p4 = loglog(w_D, mean_psd_D1, '-',  'Color', 'r');
p5 = loglog(w_D, mean_psd_D2, '--', 'Color', 'm');

p6 = loglog(w_XY, psd_m1*2, 'k-.');
loglog(w_XY, psd_m1/2, 'k-.');

set(gca, 'GridColor', [0.9 0.9 0.9]);
xlabel(['Wavenumber (cycles / ', units, ')'])
ylabel(['Power (', units, '^2 / cy / ', units, ')'])
title('Output Directional PSDs')
leg = {'Range','X-Lon','Y-Lat','\-Diag','/-Diag','+/- 3 dB'};
LH = legend([p1 p2 p3 p4 p5 p6], leg, ...
    'Location','northeast','Interpreter','none');
LH.ItemTokenSize = [15 18];

[pow1, int1] = Power_Fit_Coeffs(w_XY, psd_m1, CutOff);
[pow2, int2] = Power_Fit_Coeffs(w_D, psd_m2, CutOff);

RMSd = UCon * rms(Z,'all');

RMSfc = sqrt(int1/(pow1+1) * ...
    ((sr/2)^(pow1+1) - (1/CutOff)^(pow1+1))) * UCon;

str = {sprintf('Full RMS = %0.2f %s', RMSd, Urms)};
if CutOff < 9999
    str = [str; sprintf('%0d %s RMS = %0.2f %s', ...
        CutOff, units, RMSfc, Urms)];
end
str = [str; sprintf('X/Y: G_x_x = %4.2E*w^{%0.2f}', int1, pow1)];
str = [str; sprintf('Dia: G_x_x = %4.2E*w^{%0.2f}', int2, pow2)];

text(min(w_XY)*1.25, 1.0E-8, str, ...
    'Interpreter','tex','VerticalAlignment','bottom');

wPlotMin = min([w_X(:); w_Y(:); w_D(:)]);
wPlotMax = max([w_X(:); w_Y(:)]);
xlim([wPlotMin wPlotMax]);

switch units
    case 'ft'
        ylim([1.0E-8 100])
    case 'm'
        ylim([1.0E-8 1])
end

drawnow


%%  Coherence graph - 4th axis - RECTANGULAR SAFE

fprintf('Calculating Directional Coherence\n')

subplot(3,2,4);

% X/Y coherence with independent nFFT values
TW_ct_XY = round(TW * sr);

if TW_ct_XY >= m || TW_ct_XY >= n
    error('TrackWidth is too large for the selected DEM dimensions.');
end

% Profiles running along X, separated in Y
X1 = Z(1:(end-TW_ct_XY), :)';
X2 = Z((TW_ct_XY+1):end, :)';

% Profiles running along Y, separated in X
Y1 = Z(:, 1:(end-TW_ct_XY));
Y2 = Z(:, (TW_ct_XY+1):end);

[XCoh, w_CX] = mscohere(X1, X2, nFFT_X, ovlp_X, nFFT_X, sr);
[YCoh, w_CY] = mscohere(Y1, Y2, nFFT_Y, ovlp_Y, nFFT_Y, sr);

XCoh = XCoh(2:end-1,:);
YCoh = YCoh(2:end-1,:);
w_CX = w_CX(2:end-1);
w_CY = w_CY(2:end-1);

clear X1 X2 Y1 Y2


% Diagonal coherence from real diagonal profiles.
% Adjacent diag(Z,k) lines are dx/sqrt(2) apart perpendicular to the line.
diagShift = max(1, round(TW * sr * sqrt(2)));

D1Coh = [];
D2Coh = [];
w_CD_full = [];

for ii = 1:numel(validDiagOffsets)

    targetOffset = validDiagOffsets(ii) + diagShift;
    jj = find(validDiagOffsets == targetOffset, 1);

    if isempty(jj)
        continue;
    end

    a1 = D1cells{ii};
    a2 = D1cells{jj};
    b1 = D2cells{ii};
    b2 = D2cells{jj};

    La = min(numel(a1), numel(a2));
    Lb = min(numel(b1), numel(b2));

    if La >= requiredDiagSamples
        s1 = floor((numel(a1)-La)/2) + 1;
        s2 = floor((numel(a2)-La)/2) + 1;
        aa1 = a1(s1:s1+La-1);
        aa2 = a2(s2:s2+La-1);

        [ctmp, wtmp] = mscohere(aa1, aa2, ...
            nFFT_D, ovlp_D, nFFT_D, dsr);

        D1Coh(:,end+1) = ctmp; %#ok<AGROW>
        w_CD_full = wtmp;
    end

    if Lb >= requiredDiagSamples
        s1 = floor((numel(b1)-Lb)/2) + 1;
        s2 = floor((numel(b2)-Lb)/2) + 1;
        bb1 = b1(s1:s1+Lb-1);
        bb2 = b2(s2:s2+Lb-1);

        [ctmp, wtmp] = mscohere(bb1, bb2, ...
            nFFT_D, ovlp_D, nFFT_D, dsr);

        D2Coh(:,end+1) = ctmp; %#ok<AGROW>
        w_CD_full = wtmp;
    end
end

if isempty(D1Coh) || isempty(D2Coh)
    warning(['Not enough valid diagonal pairs for diagonal coherence. ' ...
             'Diagonal coherence curves will not be plotted.']);
    hasDiagCoh = false;
    w_CD = [];
else
    hasDiagCoh = true;
    D1Coh = D1Coh(2:end-1,:);
    D2Coh = D2Coh(2:end-1,:);
    w_CD = w_CD_full(2:end-1);
end


% Common X/Y coherence grid
wMinC = max(min(w_CX), min(w_CY));
wMaxC = min(max(w_CX), max(w_CY));

if wMaxC <= wMinC
    error('No common X/Y wavenumber interval for coherence.');
end

nCommonC = min(numel(w_CX), numel(w_CY));
w_C = linspace(wMinC, wMaxC, nCommonC)';

XCoh_common = interp1(w_CX, XCoh, w_C, 'linear');
YCoh_common = interp1(w_CY, YCoh, w_C, 'linear');

Xmean = mean(XCoh_common,2);
Ymean = mean(YCoh_common,2);
Xstd  = std(XCoh_common,[],2);
Ystd  = std(YCoh_common,[],2);

MCoh = (Xmean + Ymean)/2;
Mstd = (Xstd + Ystd)/2;

allCoh = [XCoh_common, YCoh_common];
cohMin = min(allCoh,[],2);
cohMax = max(allCoh,[],2);

p1 = semilogx(w_C, cohMin, 'Color', [.8 .8 .8]);
hold on; grid on; box on;

fill([w_C; flipud(w_C)], ...
     [cohMin; flipud(cohMax)], ...
     [.8 .8 .8], 'EdgeColor', [.8 .8 .8]);

p2 = semilogx(w_CX, mean(XCoh,2), '-',  'Color', 'b');
p3 = semilogx(w_CY, mean(YCoh,2), '--', 'Color', 'g');

if hasDiagCoh
    p4 = semilogx(w_CD, mean(D1Coh,2), '-',  'Color', 'r');
    p5 = semilogx(w_CD, mean(D2Coh,2), '--', 'Color', 'm');
else
    p4 = semilogx(NaN,NaN,'-',  'Color','r');
    p5 = semilogx(NaN,NaN,'--', 'Color','m');
end

p6 = semilogx(w_C, MCoh + Mstd, 'k-.');

xlabel(['Wavenumber (cycles / ', units, ')'])
ylabel('Directional Coherence')
title(sprintf('Output Track Coherence, %.1f %s Width', TW, units))

LH = legend([p1 p2 p3 p4 p5 p6], ...
    {'Range','X-Lon','Y-Lat','\-Diag','/-Diag','\mu +1\sigma'});
LH.ItemTokenSize = [15 18];

Ymax = max([1; MCoh + Mstd]);
Ymax = min(1.2, ceil(Ymax*10)/10);
ylim([0 Ymax])
xlim([min(w_C) max(w_C)])

drawnow


%% Loop through data for RMS Grid

fprintf( 'Calculating RMS Grid\n' )

% Gaussian detrending - RMS surface
switch units
    case 'ft'
        Swin = 32.8 * sr;
    case 'm'
        Swin = 10 * sr;
end
ConMat = Gaussian_filter( floor(Swin/2), Swin/3 );
Zsm  = conv2( Z, ConMat, 'same');       %  Smoothed surface
Zrms = Z - Zsm;                         %  RMS Detrended Surface
clear Zsm

% % True RMS window size and independent step.
% win_sz = max(1, round(Swin));
% st_sz  = max(1, floor(win_sz*(1-overlap)));
% 
% R_blocks = max(0, floor((m-win_sz)/st_sz) + 1);
% C_blocks = max(0, floor((n-win_sz)/st_sz) + 1);
% 
% if R_blocks == 0 || C_blocks == 0
%     error('Selected DEM is too small for the requested RMS window.');
% end
% 
% rmsgrid = zeros(R_blocks, C_blocks);
% 
% for row = 1:R_blocks
%     r1 = 1 + (row-1)*st_sz;
%     r2 = r1 + win_sz - 1;
% 
%     for col = 1:C_blocks
%         c1 = 1 + (col-1)*st_sz;
%         c2 = c1 + win_sz - 1;
% 
%         submat = Zrms(r1:r2, c1:c2) * UCon;
%         rmsgrid(row,col) = rms(submat(:));
%     end
% end

% --- AUTOMATIC WINDOW RESIZING FIX ---
win_sz = max(1, round(Swin));
if win_sz > m || win_sz > n
    warning('Analysis window (%d px) is larger than DEM matrix (%dx%d). Auto-scaling window to fit DEM.', win_sz, m, n);
    win_sz = max(1, min([m, n])); % Cap window to maximum allowable matrix dimension
end

ConMat = Gaussian_filter( floor(win_sz/2), win_sz/3 );
Zsm  = conv2( Z, ConMat, 'same');       % Smoothed surface
Zrms = Z - Zsm;                         % RMS Detrended Surface
clear Zsm

% Calculate safe step size and block counts
st_sz = max(1, floor(win_sz * (1 - overlap)));

R_blocks = max(1, floor((m - win_sz) / st_sz) + 1);
C_blocks = max(1, floor((n - win_sz) / st_sz) + 1);
fprintf('  -> Grid size: %dx%d | Window size: %d | Blocks: %dx%d\n', m, n, win_sz, R_blocks, C_blocks);

rmsgrid = zeros(R_blocks, C_blocks);
for row = 1:R_blocks
    r1 = 1 + (row-1)*st_sz;
    r2 = r1 + win_sz - 1;
    for col = 1:C_blocks
        c1 = 1 + (col-1)*st_sz;
        c2 = c1 + win_sz - 1;
        submat = Zrms(r1:r2, c1:c2) * UCon;
        rmsgrid(row,col) = rms(submat(:));
    end
end

%  Graph RMS Grid
subplot( 3, 2, 5 )
if ismatrix(rmsgrid) && ~isvector(rmsgrid)
    pcolor( rmsgrid );
    shading faceted
else
    imagesc( rmsgrid );
    axis tight;
    set(gca, 'YDir', 'normal');
end
% camlight
ylabel( 'Grid Square' );
xlabel( sprintf( '%0.1f > DetRMS (%s) > %0.1f, CF = %0.1f', ...
    min(min(rmsgrid)), Urms, max(max(rmsgrid)), max(max(Zrms))/rms(Zrms,'all') ) )
% Crest Factor = max( detrended surface ) / RMS( detrended surface )
switch units
    case 'ft'
title( sprintf('Output DETRENDED RMS Map\n%.1fx%.1f %s Grid, %.0f%% Overlap', ...
        Swin/sr, Swin/sr, units, overlap*100 ))
    case 'm'
title( sprintf('Output DETRENDED RMS Map\n%.0fx%.0f %s Grid, %.0f%% Overlap', ...
        Swin/sr, Swin/sr, units, overlap*100 ))
end


clear  rmsgrid  Zrms ConMat  submat  ntu  R_blocks  C_blocks c1 c2 col Ccnt ...
    dstr r1 r2 row Rcnt st_sz 

drawnow

%% Loop through data for stationarity histograms
%

fprintf( 'Calculating PDF Stationarity\n' )

cnt = 1;                                            % loop count
if CutOff<9999
    Swin = CutOff*sr;          %  Standardize, or link to CutOff?
else
    %  use 1 arcsec NTU size
    switch units
        case 'ft'
            Swin = 32.8*sr;
        case 'm'
            Swin = 10*sr;
    end
end

map = m*n;
block = floor(Swin);                                    % block size in pts

% --- AUTOMATIC BLOCK RESIZING FIX FOR NARROW ROADS ---
if block > m || block > n
    warning('Stationarity block size (%d px) is larger than DEM dimensions (%dx%d). Auto-scaling block to fit DEM.', block, m, n);
    block = max(1, min([m, n])); 
end

% Safe max elevation calculation (handles zero/NaN edge cases)
valid_Z = abs(Z(:) * UCon);
valid_Z = valid_Z(~isnan(valid_Z));
if isempty(valid_Z) || max(valid_Z) == 0
    max_int = 1.0; 
else
    max_int = floor(max(valid_Z));
    if max_int == 0, max_int = 1.0; end
end

bwidth = 2 * max_int / 101;                             % 100 bins
if bwidth == 0, bwidth = 0.01; end                      % Prevent zero bin width
bins = -max_int : bwidth : max_int;                     % histogram bin vector

if overlap > 1
    overlap = overlap / 100;
end

st_sz = max(1, floor(block * (1 - overlap)));
R_blocks = max(1, floor((m - block) / st_sz) + 1);
C_blocks = max(1, floor((n - block) / st_sz) + 1);

if R_blocks == 0 || C_blocks == 0
    error('Selected DEM is too small for the stationarity window.');
end

subplot( 3, 2, 6 )
[ Rpdf, ~ ] = histcounts( reshape( Z*UCon, 1, map ), bins,...
    'Normalization', 'pdf' );
hold on

% Gaussian detrending - PDF surface
% Swin = 32;
if exist( 'sigma', 'var' )==0
    sigma = RMSfc;  % sigma is the requested RMS, RMSf is the true value
end

ConMat = Gaussian_filter( floor(block/2), Swin*sigma );
Zsm  = conv2( Z, ConMat, 'same');       %  Smoothed surface
Zsm = Z - Zsm;                         %  PDF Detrended Surface
% Zpdf = Z;
% clear Zsm

[ DTpdf, ~ ] = histcounts( reshape( Zsm*UCon, 1, map ), bins,...
    'Normalization', 'pdf' );
Pnorm = max( DTpdf )*1.5;

Rcdf = cumsum(Rpdf)*bwidth;
DTcdf = cumsum(DTpdf)*bwidth;

for row = 1: 1: R_blocks
    r1 = 1 + (row-1)*st_sz;
    r2 = r1 + block-1;
    % r2 = r1 + st_sz - 1;
    for col = 1: 1: C_blocks
        %  NTU RMS  ===============
        c1 = 1 + (col-1)*st_sz;
        c2 = c1 + block-1;
        % c2 = c1 + st_sz - 1;
        submat = Zsm( r1:r2, c1:c2 )*UCon;

        % calculate histogram
        [ HGpdf, ~ ]  = histcounts( reshape(submat, 1, []), bins,...
            'Normalization', 'pdf' );
        
        HGcdf = cumsum(HGpdf)*bwidth;
        DDmax = max( abs( DTcdf - HGcdf ) );
        DDmat(row, col) = DDmax;

        p1 = plot( bins(2:end)-bwidth/2, HGpdf/Pnorm+1, ...
            'Color', [0.7,0.7,0.7], 'LineWidth', 0.5 );
        plot( bins(2:end)-bwidth/2, HGcdf, ...
            'Color', [0.7,0.7,0.7], 'LineWidth', 0.5 );
        cnt = cnt + 1;
        clear HGpdf
    end
end

%  Graph PDF collection
subplot( 3, 2, 6 )

plot( bins(2:end)-bwidth/2, DTcdf, 'b', 'LineWidth', 1.5 );
plot( bins(2:end)-bwidth/2, Rcdf,  'r:', 'LineWidth', 2 );
p2 = plot( bins(2:end)-bwidth/2, DTpdf/Pnorm+1, 'b', 'LineWidth', 1.5 );
p3 = plot( bins(2:end)-bwidth/2, Rpdf/Pnorm+1,  'r:', 'LineWidth', 2 );

axis( 'tight' )
ylim( [ 0 2 ] )
xl = xlim;
yl = ylim;
line( [0,0], yl, 'Color', 'k', 'LineWidth', 1 );
line( xl, [1 1], 'Color', 'k', 'LineWidth', 1 );
LegStr = { 'DET\newlineGrid\newline.', 'DET\newlineMap\newline.', 'RAW\newlineMap' };
LH = legend( [p1 p2 p3], LegStr, 'Location', 'eastoutside', 'Interpreter', 'tex' );
LH.ItemTokenSize = [ 15 18 ];
LH.Box = 'off';
LH.BackgroundAlpha = 0;

box on;  grid on;
xlabel( sprintf( 'Elevation (%s)', Urms ) )

set( gca, 'YTick', [] )
ylabel( 'CDF     |     PDF' )
title( sprintf('Output DETRENDED Stationarity\n%.0fx%.0f %s Window, %.0f%% Overlap', ...
    Swin/sr, Swin/sr, units, overlap*100 ) )


text( xl(2) / 2, 0.35, {'\DeltaCDF'; 'max ='; ...
    num2str(round(max(DDmat(:))*10000)/10000)}, ...
    'interpreter', 'tex', 'HorizontalAlignment', 'center' );

clear cnt block max_int bwidth bins overlap st_sz R_blocks C_blocks...
    subhist rmsgrid Rpdf Zrms ConMat DTpdf submat y1 ntu c1 c2 col ...
    dstr DDmat DDmax DTcdf HGcdf LegStr map Pnorm r1 r2 row Rcdf ...
    Swin xl yl Zsm 

drawnow


%% Print report
%  Save statistical report (append to previous surface graphs)
disp( ' ' )
str = input( 'Press enter to continue or [Y/y] to save report to PDF > ', 's' );
if contains( lower(str), 'y' )
    fprintf( 'Printing Report to PDF\n' )

    % Define cross-platform directory and file paths
    outDir = fullfile(pwd, 'AVT_Out');
    pdfPath = fullfile(outDir, [PSname, '.pdf']);

    % Create folder to save output GIS data if it doesn't exist
    if ~exist( outDir, 'dir' )
        mkdir( outDir );
    end

    exportgraphics( f2, pdfPath, 'Append', true, 'Padding', 'tight' );

    % Optional cross-platform open command (replaces winopen):
    % open( pdfPath ); 
end
close( f2 );

%  Give second chance to save data to output file here
if numel( Fext ) == 0

    disp( ' ' )
    disp( 'Save data to output file?  Options:' );
    disp( '  0 - none (or "N")' );
    disp( '  1 - MATLAB (MAT) file' );
    disp( '  2 - Pointcloud (LAS) file' );
    disp( '  3 - GeoTIFF (TIF) file' );
    disp( '  4 - ASCII (xyz) file' );
    disp( '  5 - Curved Regular Grid (CRG) file' );
    disp( '  6 - ADAMS Road (RDF) file' );
    disp( '  7 - Polygon File format (PLY) file' );
    disp( '  example input:  1,3' );
    disp( ' ' );
    
    str = input( 'input: > ', 's' );
    
    Fext = {};
    if strfind( str, '1' )>0;  Fext = [ Fext, 'mat' ];  fprintf( '%s.mat\n', PSname );  end
    if strfind( str, '2' )>0;  Fext = [ Fext, 'las' ];  fprintf( '%s.las\n', PSname );  end
    if strfind( str, '3' )>0;  Fext = [ Fext, 'tif' ];  fprintf( '%s.tif\n', PSname );  end
    if strfind( str, '4' )>0;  Fext = [ Fext, 'xyz' ];  fprintf( '%s.xyz\n', PSname );  end
    if strfind( str, '5' )>0;  Fext = [ Fext, 'crg' ];  fprintf( '%s.crg\n', PSname );  end
    if strfind( str, '6' )>0;  Fext = [ Fext, 'rdf' ];  fprintf( '%s.rdf\n', PSname );  end
    if strfind( str, '7' )>0;  Fext = [ Fext, 'ply' ];  fprintf( '%s.ply\n', PSname );  end
end

%  Fext = [ Fext, 'mat', 'las', 'tif' };%, 'xyz',  'crg', 'rdf' }


%%  MAT-file export

if max( ismember( Fext, 'mat') )
    disp(' ')
    fprintf( 'Exporting MAT Data\n' )

    varargin.Z = Z;

    %  Create folder to save output GIS data
    if ~exist( [ pwd, '\AVT_Out\' ], 'dir' )
        mkdir( [ pwd, '\AVT_Out\' ] );
    end

    save( ['AVT_Out\', PSname,'.mat'], 'varargin' );
    fprintf( '%s is ready\n', [PSname,'.mat'] )
end


%%  XYZ export

if max( ismember( Fext, 'xyz') )
    disp(' ')
    fprintf( 'Exporting CSV Data\n' )
    
    [m,n] = size(Z);   % m = Y rows, n = X columns
    x = (0:n-1) * PixelWidth;
    y = (0:m-1) * PixelWidth;
    [X,Y] = meshgrid(x,y);

    XX = X(:);
    YY = Y(:);
    zz = Z(:);
    
    %  Create folder to save output GIS data
    if ~exist( [ pwd, '\AVT_Out\' ], 'dir' )
        mkdir( [ pwd, '\AVT_Out\' ] );
    end

    writematrix( [ XX, YY, zz ], [ 'AVT_Out\', PSname, '.csv' ] )
    fprintf( '%s is ready\n', [PSname,'.csv'] )

    % cleanup workspace for RAM
    clear x y X Y XX YY zz
end


%%  Create LAS structure

% if max( ismember( Fext, 'las') )
%     if ~exist( 'LASwrite.m', 'file' )
%         disp(' ')
%         disp( '"LAS-IO" code not found - download at:' )
%         url = 'https://www.mathworks.com/matlabcentral/fileexchange/66033-lasio';
%         fprintf( 1,'<a href="matlab: web(''%s'') ">%s</a>\n', url, url );
% 
%        ind = ismember( Fext, 'las');
%        Fext( ind ) = '';
%     end
% end

%%  Check and initialize LAS-IO path dynamically
if max(ismember(Fext, 'las'))
    if ~exist('LASwrite.m', 'file')
        try
            disp(' ')
            disp('Searching for LAS-IO path for initialization...')
            CP = pwd;

            % Search recursively for LASwrite.m starting from current directory
            List = dir(fullfile(CP, '**', 'LASwrite.m'));

            % Walk up parent directory tree if not found in pwd
            while isempty(List)
                CPnew = fileparts(CP);
                if strcmp(CPnew, CP)
                    error('LAS-IO directory containing LASwrite.m not found.');
                end
                CP = CPnew;
                List = dir(fullfile(CP, '**', 'LASwrite.m'));
            end

            % Add the directory containing LASwrite.m to MATLAB path
            lasFolder = List(1).folder;
            addpath(lasFolder);
            disp('LAS-IO found and added to MATLAB path.')

        catch ME
            disp(' ')
            disp('"LAS-IO" code not found - download at:')
            url = 'https://www.mathworks.com/matlabcentral/fileexchange/66033-lasio';
            fprintf(1, '<a href="matlab: web(''%s'') ">%s</a>\n', url, url);
            fprintf('LAS initialization failed: %s\n', ME.message);

            % Safely remove 'las' from requested export list to prevent crashes
            ind = ismember(Fext, 'las');
            Fext(ind) = [];
        end
    end
end

if max( ismember( Fext, 'las') )
    disp(' ')
    fprintf( 'Exporting LAS Data\n' )

    %  create X/Y matrices - m = Y rows, n = X columns
    x = (0:n-1) * PixelWidth;
    y = (0:m-1) * PixelWidth;
    [X,Y] = meshgrid(x,y);
    pts = m*n;

    gen_sft = matlabRelease;
    gen_sft = sprintf( 'MATLAB %s: AVT380-TA1 3D Terrain Report (units=%s)',...
                        gen_sft.Release, units );
    gen_sft = gen_sft(1:min(end,64)); 
    SF = 1000;
    
    header = struct( 'file_signature', 'LASF',...
                    'file_source_id', 2025,...
                    'global_encoding_gps_time_type', 0,...
                    'global_encoding_reserved', 0,...
                    'project_id_1', 0,...
                    'project_id_2', 0,...
                    'project_id_3', 0,...
                    'project_id_4', PSname(7:min(end,7+16)),... % 16 characters
                    'version_major', 1,...
                    'version_minor', 4,...
                    'system_identifier','PDAL',...
                    'generating_software', gen_sft,... % 64 characters
                    'file_creation_day', day(datetime('now'),'dayofyear'),...
                    'file_creation_year', year(datetime('now')),...
                    'header_size', [],...
                    'offset_to_data', [],...
                    'n_variable_length_records', 0,...
                    'point_data_format_id', '6',... % 1-10, 2 characters
                    'point_data_record_length', 30,... % appears to be 2X the number of fields selected in the format ID
                    'n_point_records', pts,...
                    'n_points_by_return',[pts;0;0;0;0],...
                    'x_scale_factor', 1/SF,...
                    'y_scale_factor', 1/SF,...
                    'z_scale_factor', 1/SF,...
                    'x_offset', 0,...
                    'y_offset', 0,...
                    'z_offset', 0,...
                    'max_x', max( max(X) ),...
                    'min_x', min( min(X) ),...
                    'max_y', max( max(Y) ),...
                    'min_y', min( min(Y) ),...
                    'max_z', max( max(Z) ),...
                    'min_z', min( min(Z) ) );    
                    % 'header_size', 227,...
                    % 'offset_to_data', 227,...

    %  Type 6
    record = struct('x', reshape( X, pts, 1 ),... % uint32 vector
                    'y', reshape( Y, pts, 1 ),...% uint32 vector
                    'z', reshape( Z, pts, 1 ),...% uint32 vector
                    'intensity', ones(pts,1),...
                    'return_number', ones(pts,1),...
                    'number_of_returns', ones(pts,1),...
                    'classification_flag_1', zeros(pts,1),...
                    'scanner_channel', ones(pts,1),...
                    'scan_direction_flag', zeros(pts,1),...
                    'flightline_edge_flag', zeros(pts,1),...
                    'classification', zeros(pts,1),... %0=created, 1=unclassified, 2=ground
                    'user_data', zeros(pts,1),...
                    'scan_angle', zeros(pts,1),...
                    'point_source_id', zeros(pts,1), ...
                    'gps_time', zeros(pts,1) ...
                    );  %  Change point_data_format_id to 2 if RGB included,...
                    % 'red',...
                    % 'green',...
                    % 'blue');
    
    S = struct( 'header', header, 'record', record );
    
    %  Create folder to save output GIS data
    if ~exist( [ pwd, '\AVT_Out\' ], 'dir' )
        mkdir( [ pwd, '\AVT_Out\' ] );
    end

    LASwrite( S, ['AVT_Out\', PSname,'.las'], 'recordFormat', 6, 'verbose', false );
    % winopen( [PSname,'.las'] )
    fprintf( '%s is complete\n', [PSname,'.las'] )

    clear x y X Y pts gen_sft SF header record S 
end



%%  GEOTIFF export


% if max( ismember( Fext, 'tif') )
%     if ~exist( 'geotiffwrite.m', 'file' )
%         disp(' ')
%         disp( '"GeoTIFFwrite" code not found - download at:' )
%         url = 'https://www.mathworks.com/matlabcentral/fileexchange/27959-geotiffwrite';
%         fprintf( 1,'<a href="matlab: web(''%s'') ">%s</a>\n', url, url );
% 
%        ind = ismember( Fext, 'tif');
%        Fext( ind ) = '';
%     end
% end
%% Check and initialize GeoTIFF path dynamically
if max(ismember(Fext, 'tif'))
    if ~exist('geotiffwrite.m', 'file')
        try
            disp(' ')
            disp('Searching for GeoTIFF code path for initialization...')
            CP = pwd;

            % Search recursively for geotiffwrite.m starting from current directory
            List = dir(fullfile(CP, '**', 'geotiffwrite.m'));

            % Walk up parent directory tree if not found in pwd
            while isempty(List)
                CPnew = fileparts(CP);
                if strcmp(CPnew, CP)
                    error('GeoTIFF directory containing geotiffwrite.m not found.');
                end
                CP = CPnew;
                List = dir(fullfile(CP, '**', 'geotiffwrite.m'));
            end

            % Add the directory containing geotiffwrite.m to MATLAB path
            tifFolder = List(1).folder;
            addpath(tifFolder);
            disp('GeoTIFF code found and added to MATLAB path.')

        catch ME
            disp(' ')
            disp('"GeoTIFFwrite" code not found - download at:')
            url = 'https://www.mathworks.com/matlabcentral/fileexchange/27959-geotiffwrite';
            fprintf(1, '<a href="matlab: web(''%s'') ">%s</a>\n', url, url);
            fprintf('GeoTIFF initialization failed: %s\n', ME.message);

            % Safely remove 'tif' from requested export list to prevent crashes
            ind = ismember(Fext, 'tif');
            Fext(ind) = [];
        end
    end
end

if max( ismember( Fext, 'tif') )
    disp(' ')
    fprintf( 'Exporting GeoTIFF Data\n' )
    % make_option;
    % option.NaN = -32768;
    % option.GeographicTypeGeoKey = 4267;
    % option.GTCitationGeoKey = 'Geographic Model/GeoTIFF 1.0';
    % option.GeogCitationGeoKey = 'NAD 27 Datum';
    switch units
        case 'ft'
            opt.GeogCitationGeoKey = Froot;
            opt.GTModelTypeGeoKey = 1;
            opt.GTRasterTypeGeoKey = 1;
            opt.ModelPixelScaleTag = [1/sr;1/sr;1];
            opt.ModelTiepointTag = [0;0;0;0;0;0];
            opt.GeographicTypeGeoKey = 4269;
            opt.GeogGeodeticDatumGeoKey = 6269;
            opt.GeogPrimeMeridianGeoKey = 8901;
            opt.GeogLinearUnitsGeoKey = 9001;
            opt.GeogAngularUnitsGeoKey = 9102;
            opt.GeogEllipsoidGeoKey = 7019;
            opt.ProjectedCSTypeGeoKey = 102709;
            opt.ProjectionGeoKey = 12733;
            opt.ProjCoordTransGeoKey = 1;
            opt.ProjLinearUnitsGeoKey = 9003;
            opt.VerticalCSTypeGeoKey = 5103;
            opt.VerticalUnitsGeoKey = 9003;
            % opt.GeogCitationGeoKey = '';
        case 'm'
            % meters
            opt.GeogCitationGeoKey = Froot;
            opt.GTModelTypeGeoKey = 1;
            opt.GTRasterTypeGeoKey = 1;
            opt.ModelPixelScaleTag = [1/sr;1/sr;1];
            opt.ModelTiepointTag = [0;0;0;0;0;0];
            opt.GeographicTypeGeoKey = 4269;
            opt.GeogGeodeticDatumGeoKey = 6269;
            opt.GeogPrimeMeridianGeoKey = 8901;
            opt.GeogLinearUnitsGeoKey = 9001;
            opt.GeogAngularUnitsGeoKey = 9102;
            opt.GeogEllipsoidGeoKey = 7019;
            opt.ProjectedCSTypeGeoKey = 32611;
            opt.ProjectionGeoKey = 12733;
            opt.ProjCoordTransGeoKey = 1;
            opt.ProjLinearUnitsGeoKey = 9001;
            opt.VerticalCSTypeGeoKey = 5019;
            opt.VerticalUnitsGeoKey = 9001;
    end
 
    % geotiffwrite( [PSname,'.tif'], [ 0, 0; Ly, Lx ], Z );


    %  Create folder to save output GIS data
    % if ~exist( [ pwd, '\AVT_Out\' ], 'dir' )
    %     mkdir( [ pwd, '\AVT_Out\' ] );
    % end
    % geotiffwrite( ['AVT_Out\', PSname,'.tif'], [ 0, 0; Ly, Lx ], Z, 32, opt );
    % fprintf( '%s is complete\n', [PSname,'.tif'] )
    % 
    % clear opt 
    % Cross-platform directory pathing (Windows / Linux / macOS)
    outDir = fullfile(pwd, 'AVT_Out');
    if ~exist( outDir, 'dir' )
        mkdir( outDir );
    end
    tifPath = fullfile(outDir, [PSname, '.tif']);

    gisStruct = struct();

    if isstruct(varargin)
        gisStruct = varargin;
    elseif iscell(varargin) && ~isempty(varargin)
        firstArg = varargin{1};
        if isstruct(firstArg)
            gisStruct = firstArg;
        elseif iscell(firstArg) && ~isempty(firstArg) && isstruct(firstArg{1})
            gisStruct = firstArg{1};
        end
    end

    wktStr = '';
    if isfield(gisStruct, 'wkt') && ~isempty(gisStruct.wkt)
        wktStr = gisStruct.wkt;
    elseif exist('wkt', 'var') && ~isempty(wkt)
        wktStr = wkt;
    end

    % Parse EPSG Code (default to 32633 for UTM Zone 33N if WKT parsing is unavailable)
    epsgCode = 32633;
    if ~isempty(wktStr)
        epsgTokens = regexp(wktStr, 'AUTHORITY\["EPSG","(\d+)"\]', 'tokens');
        if ~isempty(epsgTokens)
            epsgCode = str2double(epsgTokens{end}{1});
        end
    end

    % --- 2. EXPORT GEOTIFF OR TIFF FALLBACK ---
    try
        if exist('maprefcells', 'file')
            R = maprefcells([0, Ly], [0, Lx], size(Z));

            % Assign ProjectedCRS if supported
            if exist('wkt2crs', 'file') && ~isempty(wktStr)
                try R.ProjectedCRS = wkt2crs(wktStr); catch; end
            elseif exist('projcrs', 'file')
                try R.ProjectedCRS = projcrs(epsgCode); catch; end
            end

            % Write GeoTIFF with explicit CoordRefSysCode to satisfy Mapping Toolbox
            try
                geotiffwrite(tifPath, Z, R, 'CoordRefSysCode', epsgCode);
            catch
                geotiffwrite(tifPath, Z, R);
            end
        else
            % Legacy File Exchange geotiffwrite syntax fallback
            geotiffwrite(tifPath, [0, 0; Ly, Lx], Z, 32, opt);
        end
        fprintf('  -> %s exported successfully.\n', [PSname, '.tif']);

    catch ME
        warning('GeoTIFF export driver fallback: %s', ME.message);

        % Safe 32-bit Float TIFF Export using MATLAB's native Tiff class (Replaces imwrite)
        try
            t = Tiff(tifPath, 'w');
            t.setTag('ImageLength', size(Z, 1));
            t.setTag('ImageWidth', size(Z, 2));
            t.setTag('Photometric', Tiff.Photometric.MinIsBlack);
            t.setTag('BitsPerSample', 32);
            t.setTag('SamplesPerPixel', 1);
            t.setTag('SampleFormat', Tiff.SampleFormat.IEEEFP);
            t.setTag('PlanarConfiguration', Tiff.PlanarConfiguration.Chunky);
            t.write(single(Z));
            t.close();
            fprintf('  -> Standard 32-bit float TIF exported: %s\n', [PSname, '.tif']);
        catch TiffErr
            warning('Failed to write TIFF fallback: %s', TiffErr.message);
        end
    end
    clear R outDir tifPath wktStr epsgTokens gisStruct epsgCode
end


%%  CRG Export

% =========================================================
% CURVED OpenCRG EXPORT FROM USER-SELECTED TRACK CENTERLINE
% =========================================================
% The user selects ordered points on the DEM. OpenCRG's own
% reference-line generators fit a smooth curve through those points,
% convert it to a regular-u curved reference line, and the DEM is then
% sampled on cross-sections normal to that reference line.

if max(ismember(Fext, 'crg'))
    % Check to see if CRG tools are initialized ==========================
    folder = 'OpenCRG';

    pathCell = regexp(path, pathsep, 'split');
    onPath = any(contains(pathCell, folder));

    if ~onPath
        try
            disp(' ')
            disp('Searching for OpenCRG path for initialization...')
            CP = pwd;
            List = dir(fullfile(CP, '**', 'OpenCRG*'));

            while isempty(List)
                CPnew = fileparts(CP);
                if strcmp(CPnew, CP)
                    error('OpenCRG directory not found.');
                end
                CP = CPnew;
                List = dir(fullfile(CP, '**', 'OpenCRG*'));
            end

            % Find crg_init.m inside the discovered OpenCRG tree.
            validDirs = List([List.isdir]);

            InitList = [];
            for k = length(validDirs):-1:1
                searchPath = fullfile(validDirs(k).folder, validDirs(k).name);
                InitList = dir(fullfile(searchPath, '**', 'crg_init.m'));
                if ~isempty(InitList)
                    break; % Found the correct OpenCRG folder tree!
                end
            end

            init_file = fullfile(InitList(1).folder, InitList(1).name);
            run(init_file);
            disp('OpenCRG found and initialized.')
        catch ME
            disp('"OpenCRG" path not found - download OpenCRG at:')
            url = 'https://www.asam.net/standards/detail/opencrg/';
            fprintf(1,'<a href="matlab: web(''%s'') ">%s</a>\n', url, url);
            fprintf('CRG initialization failed: %s\n', ME.message);

            ind = ismember(Fext, 'crg');
            Fext(ind) = [];
        end
    end
end


if max(ismember(Fext, 'crg'))
    disp(' ')
    fprintf('Exporting CURVED CRG Data\n')

    % ---------------------------------------------------------
    % 1) DEM coordinate vectors in the same orientation as Zabs
    % ---------------------------------------------------------
    % Zabs is m x n:
    %   rows    -> Y
    %   columns -> X
    xDEM_crg = (0:n-1) / sr;
    yDEM_crg = (0:m-1) / sr;

    % ---------------------------------------------------------
    % 2) Select ordered reference points on the track centerline
    % ---------------------------------------------------------
    fCRG = figure('Theme','light', ...
                  'Name','OpenCRG centerline selection', ...
                  'NumberTitle','off');

    imagesc(xDEM_crg, yDEM_crg, Zabs);
    set(gca,'YDir','normal');
    axis equal tight;
    grid on; box on;
    colormap(parula);
    colorbar;
    xlabel(sprintf('X (%s)', units));
    ylabel(sprintf('Y (%s)', units));
    title({'Select CENTERLINE points in travel order', ...
           'Left = add | Right = undo | C = CLOSE LOOP | ENTER = finish open'});
    hold on;

    % Interactive point selection with live graphical feedback.
    % ginput is called one point at a time so the selected centerline is
    % visible while it is being created.
    xp = [];
    yp = [];
    closeTrack = false;

    hSel = plot(NaN, NaN, 'ro-', ...
        'LineWidth', 1.8, ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', 'y');

    % Highlight the first point separately: clicking close to it after at
    % least 3 points also closes the circuit.
    hFirst = plot(NaN, NaN, 'go', ...
        'MarkerSize', 10, ...
        'LineWidth', 1.8, ...
        'MarkerFaceColor', 'g');

    % Closing tolerance in current TerrainReport units.
    % About 3 DEM cells, with a small minimum relative to the map size.
    closeTol = max(3/sr, 0.005*max([max(xDEM_crg)-min(xDEM_crg), ...
                                        max(yDEM_crg)-min(yDEM_crg)]));

    while true
        [xClick, yClick, button] = ginput(1);

        % ENTER terminates ginput(1) and returns empty output -> open track.
        if isempty(xClick)
            break;
        end

        % C/c explicitly closes the circuit and finishes selection.
        if button == double('c') || button == double('C')
            if numel(xp) < 3
                title({'Need at least 3 points before closing the loop', ...
                       'Left = add | Right = undo | C = CLOSE LOOP | ENTER = open'});
                drawnow;
                continue;
            end
            closeTrack = true;
            set(hSel, 'XData', [xp; xp(1)], 'YData', [yp; yp(1)]);
            drawnow;
            break;
        end

        if button == 1
            % Left click near the first point closes the loop once at least
            % three points already exist. Otherwise append a new point.
            if numel(xp) >= 3 && hypot(xClick-xp(1), yClick-yp(1)) <= closeTol
                closeTrack = true;
                set(hSel, 'XData', [xp; xp(1)], 'YData', [yp; yp(1)]);
                drawnow;
                break;
            end

            xp(end+1,1) = xClick; %#ok<AGROW>
            yp(end+1,1) = yClick; %#ok<AGROW>

            if numel(xp) == 1
                set(hFirst, 'XData', xp(1), 'YData', yp(1));
            end

        elseif button == 3
            % Right click: remove the last selected point.
            if ~isempty(xp)
                xp(end) = [];
                yp(end) = [];
            end

            if isempty(xp)
                set(hFirst, 'XData', NaN, 'YData', NaN);
            else
                set(hFirst, 'XData', xp(1), 'YData', yp(1));
            end
        else
            % Ignore other mouse buttons / keys.
            continue;
        end

        set(hSel, 'XData', xp, 'YData', yp);
        title({'Select CENTERLINE points in travel order', ...
               sprintf(['%d points | Left = add | Right = undo | ' ...
                        'C = CLOSE LOOP | ENTER = open'], numel(xp))});
        drawnow;
    end

    if numel(xp) < 3
        close(fCRG);
        error('CRG export requires at least 3 centerline points.');
    end

    if closeTrack
        % Duplicate the first point only for the spline definition so the
        % fitted reference line returns to the start point.
        pxy_user = [xp(:), yp(:); xp(1), yp(1)];
        fprintf('Closed-loop CRG selected.\n');
    else
        pxy_user = [xp(:), yp(:)];
        fprintf('Open CRG selected.\n');
    end

    drawnow;

    % The red polyline above is ONLY the temporary control polygon used
    % during point selection.  Hide it before showing the fitted/final CRG
    % geometry, otherwise the straight control segments can be mistaken for
    % the actual reference-line seam.
    if isgraphics(hSel),   set(hSel,   'Visible','off'); end
    if isgraphics(hFirst), set(hFirst, 'Visible','off'); end

    % ---------------------------------------------------------
    % 3) Convert everything to meters (OpenCRG uses meters here)
    % ---------------------------------------------------------
    if strcmpi(units, 'ft')
        toMeter = 0.3048;
    else
        toMeter = 1.0;
    end

    pxy_m    = pxy_user * toMeter;
    xDEM_m   = xDEM_crg * toMeter;
    yDEM_m   = yDEM_crg * toMeter;
    Zsrc_m   = Zabs * toMeter;
    demStep_m = (1/sr) * toMeter;

    % OpenCRG default minimum/check increment: 1 mm.  Both u and v %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % increments generated below are quantized to this value so that the
    % resulting DATA structure remains internally consistent.
    crgMinInc = 1e-3;

    % ---------------------------------------------------------
    % 4) Ask for CRG strip width BEFORE choosing u resolution
    % ---------------------------------------------------------
    % Longitudinal resolution depends not only on source DEM spacing, but
    % also on the selected track curvature and requested half-width.
    defaultHalfWidth = min(5/toMeter, 0.45*min(Lx,Ly));

    halfWidth_user = input(sprintf( ...
        'CRG half-width around centerline [%0.2f %s] > ', ...
        defaultHalfWidth, units));

    if isempty(halfWidth_user)
        halfWidth_user = defaultHalfWidth;
    end

    if ~isscalar(halfWidth_user) || ~isfinite(halfWidth_user) || halfWidth_user <= 0
        error('CRG half-width must be a positive scalar.');
    end

    halfWidth_m = halfWidth_user * toMeter;

    % ---------------------------------------------------------
    % 5) Build the reference line and choose u resolution
    %    OPEN track  -> native OpenCRG smoothing
    %    CLOSED loop -> C2 PERIODIC cubic spline
    % ---------------------------------------------------------
    % For a closed circuit it is not enough that the last point coincides
    % with the first one.  The reference curve must also have periodic first
    % and second derivatives, otherwise a visible corner / curvature jump is
    % created at the seam.  CSAPE(...,'periodic') imposes exactly these C2
    % periodic end conditions.

    if closeTrack
        if exist('csape', 'file') ~= 2
            error(['Closed-loop CRG generation requires CSAPE for a C2 ' ...
                   'periodic spline. Install/enable the MATLAB spline ' ...
                   'functions (Curve Fitting Toolbox) and retry.']);
        end

        % ---- remove accidental consecutive duplicate points ------------
        dClick = hypot(diff(pxy_m(:,1)), diff(pxy_m(:,2)));
        tolPoint = max(1e-9, 1e-6*demStep_m);
        keepPoint = [true; dClick > tolPoint];
        pxy_periodic = pxy_m(keepPoint,:);

        if size(pxy_periodic,1) < 4
            error('A closed CRG requires at least 3 distinct points plus closure.');
        end

        % Ensure exact geometric closure.
        if hypot(pxy_periodic(end,1)-pxy_periodic(1,1), ...
                 pxy_periodic(end,2)-pxy_periodic(1,2)) > tolPoint
            pxy_periodic(end+1,:) = pxy_periodic(1,:);
        else
            pxy_periodic(end,:) = pxy_periodic(1,:);
        end

        % Chord-length parameter for the user-selected control points.
        dsCtrl = hypot(diff(pxy_periodic(:,1)), diff(pxy_periodic(:,2)));
        if any(dsCtrl <= tolPoint)
            error('The closed centerline contains duplicate/degenerate points.');
        end
        sCtrl = [0; cumsum(dsCtrl)];

        % Complex representation r(s) = x(s) + i*y(s).
        rCtrl = complex(pxy_periodic(:,1), pxy_periodic(:,2));

        % First periodic fit.  Position, first derivative and second
        % derivative are periodic at s = 0 / s = L.
        pp0 = csape(sCtrl.', rCtrl.', 'periodic');

        % Re-parameterize once using a dense arc-length estimate.  This
        % makes the final spline parameter much closer to true arc length,
        % which in turn makes the OpenCRG constant-u discretization much
        % more uniform around tight corners.
        nDense = max(2000, ceil(sCtrl(end) / max(demStep_m/5, 1e-6)) + 1);
        nDense = min(nDense, 200000);   % avoid excessive temporary arrays
        sDense = linspace(sCtrl(1), sCtrl(end), nDense);
        rDense = ppval(pp0, sDense);

        aDense = [0, cumsum(abs(diff(rDense)))];
        dArc = diff(aDense);
        keepArc = [true, dArc > max(1e-12, 1e-9*demStep_m)];
        aDense = aDense(keepArc);
        rDense = rDense(keepArc);

        % Force the last dense point to be numerically identical to the
        % first one before the second periodic fit.
        rDense(end) = rDense(1);

        % -------------------------------------------------------------
        % Automatic longitudinal CRG resolution from:
        %   1) source DEM resolution,
        %   2) selected-track curvature,
        %   3) requested strip half-width.
        %
        % On an offset line at lateral coordinate v, a centerline step du
        % produces approximately ds_v = (1-kappa*v)du.  The largest physical
        % spacing therefore occurs at the outer edge and is bounded by
        % (1+|kappa|*W)du.  We keep that spacing no coarser than the DEM.
        % We also limit heading rotation per longitudinal CRG segment.
        % -------------------------------------------------------------
        ppArc0 = csape(aDense, rDense, 'periodic');
        ppArc1 = fnder(ppArc0,1);
        ppArc2 = fnder(ppArc0,2);

        nCurvScan = max(4000, min(200000, 20*ceil(aDense(end)/max(demStep_m,crgMinInc))));
        uCurvScan = linspace(0,aDense(end),nCurvScan+1);
        uCurvScan(end) = [];
        dCurv  = ppval(ppArc1,uCurvScan);
        ddCurv = ppval(ppArc2,uCurvScan);
        kCurv  = imag(conj(dCurv).*ddCurv) ./ max(abs(dCurv).^3, eps);
        kappaMax = max(abs(kCurv));

        kappaWidth = kappaMax * halfWidth_m;
        if kappaWidth >= 1
            error(['Requested CRG half-width is too large for the selected ' ...
                   'track curvature: max(|kappa|)*halfWidth = %.6f >= 1. ' ...
                   'The inner offset would fold/collapse. Reduce track width ' ...
                   'or select a smoother centerline.'], kappaWidth);
        elseif kappaWidth >= 0.85
            warning(['Selected track is close to the offset-curve singularity: ' ...
                     'max(|kappa|)*halfWidth = %.6f.'], kappaWidth);
        end

        % Maximum permitted heading change over one CRG longitudinal cell.
        % This is a numerical geometry criterion, not a fixed spatial step.
        dPhiMax = deg2rad(0.5);

        if kappaMax > 1e-12
            uinc_heading = dPhiMax / kappaMax;
        else
            uinc_heading = inf;
        end

        uinc_edge = demStep_m / (1 + kappaWidth);
        uinc_target = min([demStep_m, uinc_edge, uinc_heading]);

        % Quantize DOWN to the OpenCRG minimum increment so the actual step
        % never exceeds the geometry/DEM limit selected above.
        uinc_m = floor(uinc_target/crgMinInc) * crgMinInc;
        if uinc_m < crgMinInc
            uinc_m = crgMinInc;
            warning(['Automatic u resolution hit the OpenCRG minimum increment ' ...
                     '(%.6f m).'], crgMinInc);
        end

        % Choose an integer number of equal CRG segments for one full lap.
        % Instead of changing uinc to L/N (which produced non-mm increments),
        % rescale ONLY the spline parameter.  The XY geometry is unchanged and
        % the periodic C2 conditions are preserved.
        Lperiodic = aDense(end);
        nSegClosed = max(3, round(Lperiodic/uinc_m));
        Lcrg = nSegClosed*uinc_m;

        aCRG = aDense * (Lcrg/Lperiodic);
        ppxy = csape(aCRG, rDense, 'periodic');

        % -------------------------------------------------------------
        % Put the CRG seam where the periodic spline curvature is smallest.
        % This keeps the inevitable polygonal/discrete correction away from
        % tight corners and minimizes the change required to make the first
        % and last CRG lateral cuts identical.
        % -------------------------------------------------------------
        pp1_seam = fnder(ppxy,1);
        pp2_seam = fnder(ppxy,2);
        nSeamScan = max(2000, 20*nSegClosed);
        uSeamScan = linspace(0,Lcrg,nSeamScan+1);
        uSeamScan(end) = [];
        dSeam  = ppval(pp1_seam,uSeamScan);
        ddSeam = ppval(pp2_seam,uSeamScan);
        kSeamScan = imag(conj(dSeam).*ddSeam) ./ max(abs(dSeam).^3, eps);
        [~,iBestSeam] = min(abs(kSeamScan));
        seamU = uSeamScan(iBestSeam);
        seamCurvature = kSeamScan(iBestSeam);

        % Build the closed OpenCRG reference line explicitly at the exact
        % CRG stations, starting from the low-curvature seam chosen above.
        uNodes = (0:nSegClosed)*uinc_m;
        uSplineNodes = mod(seamU + uNodes, Lcrg);
        uSplineNodes(end) = seamU;
        rNodes = ppval(ppxy, uSplineNodes);
        rNodes(end) = rNodes(1);     % exact geometric seam

        % Initial discrete headings from the closed polygon chords.
        % IMPORTANT: for a closed CRG the first and last lateral cuts must
        % have the SAME orientation.  Geometric closure of the centerline
        % alone is not enough: if p(1) ~= p(end), v ~= 0 produces a visible
        % seam even when the centerline gap is exactly zero.
        p0Closed = angle(diff(rNodes));

        % -------------------------------------------------------------
        % Enforce a CLOSED discrete OpenCRG reference line.
        %
        % We solve simultaneously for:
        %   1) zero accumulated X displacement
        %   2) zero accumulated Y displacement
        %   3) identical start/end heading (modulo 2*pi)
        %   4) identical heading increment on both sides of the seam
        %      (discrete curvature continuity)
        %
        % The correction is represented by four smooth global basis
        % functions so that the seam is not repaired by a local kink.
        % -------------------------------------------------------------
        nPhi = numel(p0Closed);
        if nPhi < 5
            error('Closed CRG requires at least 5 heading intervals.');
        end

        p0u = unwrap(double(p0Closed(:)));      % continuous heading history
        tPhi = linspace(0,1,nPhi).';

        % Smooth correction bases.
        % b3 changes the end heading relative to the start while having
        % zero endpoint slope; b4 changes the endpoint heading derivatives
        % without changing the endpoint heading values.
        b1 = cos(p0u);
        b2 = sin(p0u);
        b3 = 3*tPhi.^2 - 2*tPhi.^3;             % smoothstep: 0 -> 1
        b4 = tPhi .* (1-tPhi);                  % 0 at both ends
        Bclose = [b1 b2 b3 b4];

        % Winding number of the tangent over one complete lap.  The start
        % and end headings must be equal modulo 2*pi, not necessarily equal
        % in the unwrapped representation.
        nTurn = round((p0u(end)-p0u(1))/(2*pi));

        coeffClose = zeros(4,1);
        convergedClose = false;

        for itClose = 1:20
            pTry = p0u + Bclose*coeffClose;
            eTry = exp(1i*pTry);

            % Closure residuals.
            Fclose = zeros(4,1);
            Fclose(1) = real(uinc_m * sum(eTry));
            Fclose(2) = imag(uinc_m * sum(eTry));
            Fclose(3) = pTry(end) - pTry(1) - 2*pi*nTurn;
            Fclose(4) = (pTry(2)-pTry(1)) - ...
                        (pTry(end)-pTry(end-1));

            if norm(Fclose(1:2)) < 1e-10 && ...
               abs(Fclose(3)) < 1e-10 && ...
               abs(Fclose(4)) < 1e-10
                convergedClose = true;
                break;
            end

            % Exact Jacobian of the four constraints with respect to the
            % four correction coefficients.
            Jclose = zeros(4,4);
            for jj = 1:4
                bj = Bclose(:,jj);
                Jclose(1,jj) = real(uinc_m * sum(1i*eTry.*bj));
                Jclose(2,jj) = imag(uinc_m * sum(1i*eTry.*bj));
                Jclose(3,jj) = bj(end) - bj(1);
                Jclose(4,jj) = (bj(2)-bj(1)) - ...
                               (bj(end)-bj(end-1));
            end

            if rcond(Jclose) < 1e-12
                error(['Closed-CRG seam correction became ill-conditioned. ' ...
                       'Select a few more centerline points and retry.']);
            end

            dCoeff = -(Jclose \ Fclose);

            % Mild damping protects against an occasional large Newton step.
            if norm(dCoeff) > 0.25
                dCoeff = dCoeff * (0.25/norm(dCoeff));
            end
            coeffClose = coeffClose + dCoeff;
        end

        if ~convergedClose
            warning(['Closed-CRG seam correction reached the iteration limit. ' ...
                     'Residuals will be printed for inspection.']);
        end

        pClosedU = p0u + Bclose*coeffClose;

        % Final residual diagnostics BEFORE wrapping to [-pi,+pi].
        closureVecFinal = uinc_m * sum(exp(1i*pClosedU));
        seamHeadingResidual = pClosedU(end)-pClosedU(1)-2*pi*nTurn;
        seamTurnBeg = pClosedU(2)-pClosedU(1);
        seamTurnEnd = pClosedU(end)-pClosedU(end-1);
        seamCurvResidual = (seamTurnBeg-seamTurnEnd)/uinc_m;

        % Store the OpenCRG headings in the conventional angular range.
        pClosedD = atan2(sin(pClosedU), cos(pClosedU));

        % Force the two endpoint headings to be bit-identical after wrapping.
        % This removes the last single-precision round-off from the lateral
        % seam.  The equality is modulo 2*pi and does not affect integration.
        pClosedD(end) = pClosedD(1);

        % The four-constraint solve already closed X/Y.  After wrapping,
        % forcing p(end)=p(1) only removes numerical modulo-2*pi noise; do
        % not run a second independent XY-only correction here, because that
        % would destroy the curvature-matching constraint at the seam.
        pClosedD = pClosedD(:).';
        pClosedD(end) = pClosedD(1);
        pClosed = single(pClosedD);
        pClosed(end) = pClosed(1);               % exact also in single

        closureCorrectionDeg = rad2deg(max(abs(atan2( ...
            sin(double(pClosed)-double(p0Closed)), ...
            cos(double(pClosed)-double(p0Closed))))));

        % Diagnostics of the DISCRETE CRG seam itself.
        discreteHeadingMismatch = abs(rad2deg(atan2( ...
            sin(double(pClosed(end))-double(pClosed(1))), ...
            cos(double(pClosed(end))-double(pClosed(1))))));
        discreteCurvMismatch = abs(( ...
            atan2(sin(double(pClosed(2))-double(pClosed(1))), ...
                  cos(double(pClosed(2))-double(pClosed(1)))) - ...
            atan2(sin(double(pClosed(end))-double(pClosed(end-1))), ...
                  cos(double(pClosed(end))-double(pClosed(end-1)))) ) / uinc_m);

        crg_data = struct;
        crg_data.head = struct;
        crg_data.head.ubeg = 0.0;
        crg_data.head.uend = Lcrg;
        crg_data.head.uinc = uinc_m;
        crg_data.head.pbeg = double(pClosed(1));
        crg_data.head.pend = double(pClosed(end));
        crg_data.head.xbeg = real(rNodes(1));
        crg_data.head.ybeg = imag(rNodes(1));
        crg_data.head.xend = real(rNodes(1));
        crg_data.head.yend = imag(rNodes(1));
        crg_data.u = single([0.0 Lcrg]);
        crg_data.p = pClosed;

        rIntegratedEnd = rNodes(1) + ...
            sum(uinc_m * exp(1i*double(pClosed)));
        refErr = abs(rIntegratedEnd-rNodes(1));

        % ---- exact C2 seam diagnostics from the periodic spline ---------
        pp1 = fnder(ppxy, 1);
        pp2 = fnder(ppxy, 2);
        [ppBreaks,~,~,~,~] = unmkpp(ppxy);
        sBeg = ppBreaks(1);
        sEnd = ppBreaks(end);

        rBeg  = ppval(ppxy, sBeg);
        rEnd  = ppval(ppxy, sEnd);
        dBeg  = ppval(pp1,   sBeg);
        dEnd  = ppval(pp1,   sEnd);
        ddBeg = ppval(pp2,   sBeg);
        ddEnd = ppval(pp2,   sEnd);

        seamPosSpline = abs(rEnd-rBeg);
        seamPhiSpline = atan2(sin(angle(dEnd)-angle(dBeg)), ...
                              cos(angle(dEnd)-angle(dBeg)));

        kBeg = imag(conj(dBeg).*ddBeg) / max(abs(dBeg)^3, eps);
        kEnd = imag(conj(dEnd).*ddEnd) / max(abs(dEnd)^3, eps);
        seamCurvSpline = kEnd-kBeg;

    else
        % Open track: use the native OpenCRG smoothing, but choose the
        % longitudinal resolution from the actual smoothed-track curvature
        % and requested strip width, exactly as for a closed loop.
        ppxy = crg_gen_pxy2ppxy(pxy_m);

        pp1_open = fnder(ppxy,1);
        pp2_open = fnder(ppxy,2);
        [ppBreaksOpen,~,~,~,~] = unmkpp(ppxy);
        uOpenBeg = ppBreaksOpen(1);
        uOpenEnd = ppBreaksOpen(end);
        nCurvScan = max(2000, min(200000, ...
            20*ceil((uOpenEnd-uOpenBeg)/max(demStep_m,crgMinInc))));
        uCurvScan = linspace(uOpenBeg,uOpenEnd,nCurvScan);
        dCurv  = ppval(pp1_open,uCurvScan);
        ddCurv = ppval(pp2_open,uCurvScan);
        kCurv  = imag(conj(dCurv).*ddCurv) ./ max(abs(dCurv).^3, eps);
        kappaMax = max(abs(kCurv));

        kappaWidth = kappaMax * halfWidth_m;
        if kappaWidth >= 1
            error(['Requested CRG half-width is too large for the selected ' ...
                   'track curvature: max(|kappa|)*halfWidth = %.6f >= 1. ' ...
                   'The inner offset would fold/collapse. Reduce track width ' ...
                   'or select a smoother centerline.'], kappaWidth);
        elseif kappaWidth >= 0.85
            warning(['Selected track is close to the offset-curve singularity: ' ...
                     'max(|kappa|)*halfWidth = %.6f.'], kappaWidth);
        end

        dPhiMax = deg2rad(0.5);
        if kappaMax > 1e-12
            uinc_heading = dPhiMax / kappaMax;
        else
            uinc_heading = inf;
        end
        uinc_edge = demStep_m / (1 + kappaWidth);
        uinc_target = min([demStep_m, uinc_edge, uinc_heading]);
        uinc_m = floor(uinc_target/crgMinInc) * crgMinInc;
        if uinc_m < crgMinInc
            uinc_m = crgMinInc;
            warning(['Automatic u resolution hit the OpenCRG minimum increment ' ...
                     '(%.6f m).'], crgMinInc);
        end

        [crg_data, refErr] = crg_gen_ppxy2phi(ppxy, uinc_m);

        seamPosSpline  = NaN;
        seamPhiSpline  = NaN;
        seamCurvSpline = NaN;
    end

    % IMPORTANT:
    % Do NOT set REFLINE_CONTINUATION = 1 for the explicitly periodic loop.
    % That option asks OpenCRG to close an OPEN reference line by extending
    % its end tangents until they intersect.  Here the curve is already
    % closed and C2-periodic by construction.

    nu = numel(crg_data.p) + 1;

    fprintf('\nCurved CRG reference line:\n');
    fprintf('  Selected reference points : %d\n', size(pxy_m,1));
    fprintf('  Longitudinal samples      : %d\n', nu);
    fprintf('  u increment               : %.6f m\n', uinc_m);
    fprintf('  Reference line length     : %.3f m\n', ...
        double(crg_data.head.uend - crg_data.head.ubeg));
    fprintf('  OpenCRG integration error : %.9f m\n', refErr);

    if closeTrack
        fprintf('  Track topology             : CLOSED - C2 PERIODIC\n');
        fprintf('  Automatic seam u           : %.6f m\n', seamU);
        fprintf('  Seam spline curvature      : %.12g 1/m\n', seamCurvature);
        fprintf('  Max closure correction     : %.12g deg\n', closureCorrectionDeg);
        fprintf('  Discrete heading mismatch  : %.12g deg\n', discreteHeadingMismatch);
        fprintf('  Discrete curvature mismatch: %.12g 1/m\n', discreteCurvMismatch);
        fprintf('  Periodic spline seam gap   : %.12g m\n', seamPosSpline);
        fprintf('  Periodic tangent mismatch  : %.12g deg\n', ...
                abs(rad2deg(seamPhiSpline)));
        fprintf('  Periodic curvature mismatch: %.12g 1/m\n', ...
                abs(seamCurvSpline));

        % This is the residual seam of the discretized OpenCRG reference
        % line, not of the C2 spline itself.
        crgSeamGap = hypot(double(crg_data.head.xend-crg_data.head.xbeg), ...
                           double(crg_data.head.yend-crg_data.head.ybeg));
        fprintf('  Discrete CRG seam gap      : %.9f m\n', crgSeamGap);
    else
        fprintf('  Track topology             : OPEN\n');
    end

    % ---------------------------------------------------------
    % 6) Choose lateral CRG resolution independently from u
    % ---------------------------------------------------------
    % Lateral sampling is tied to the actual DEM spacing, not copied from
    % uinc.  Keep it no coarser than the source DEM while preserving a
    % symmetric grid and millimetre-compatible OpenCRG increments.
    demStepQuant_m = max(crgMinInc, floor(demStep_m/crgMinInc)*crgMinInc);
    halfWidthQuant_m = round(halfWidth_m/crgMinInc)*crgMinInc;

    % Minimum number of half-width intervals needed so vinc <= DEM spacing.
    nHalf = max(1, ceil(halfWidthQuant_m/demStepQuant_m));

    % Pick the closest millimetre-compatible lateral increment for that
    % number of intervals.  The resulting width differs from the requested
    % width by at most a few millimetres in normal use.
    vincSteps = round((halfWidthQuant_m/crgMinInc)/nHalf);
    vincSteps = max(1, min(vincSteps, floor(demStep_m/crgMinInc)));
    vinc_m = vincSteps * crgMinInc;

    vvec = (-nHalf:nHalf) * vinc_m;
    nv = numel(vvec);
    actualHalfWidth_m = nHalf*vinc_m;

    fprintf('  Source DEM resolution     : %.6f m\n', demStep_m);
    fprintf('  Max |centerline curvature|: %.12g 1/m\n', kappaMax);
    fprintf('  max(|kappa|)*half-width   : %.12g\n', kappaWidth);
    fprintf('  u limit from outer edge   : %.6f m\n', uinc_edge);
    if isfinite(uinc_heading)
        fprintf('  u limit from heading      : %.6f m\n', uinc_heading);
    else
        fprintf('  u limit from heading      : Inf\n');
    end
    fprintf('  Automatic u increment     : %.6f m\n', uinc_m);
    fprintf('  Requested CRG width       : %.6f m\n', 2*halfWidth_m);
    fprintf('  Transverse samples        : %d\n', nv);
    fprintf('  Automatic v increment     : %.6f m\n', vinc_m);
    fprintf('  Actual CRG width          : %.6f m\n', 2*actualHalfWidth_m);

    % ---------------------------------------------------------
    % 7) Build a temporary complete CRG so OpenCRG itself can
    %    transform every (u,v) node into consistent (x,y)
    % ---------------------------------------------------------
    crg_data.v = single(vvec);
    crg_data.z = zeros(nu, nv, 'single');
    crg_data.ct = { ...
        'Generated by TerrainReport_rectangular'; ...
        'Curved reference line selected interactively from DEM'; ...
        sprintf('Source DEM resolution: %.6f m', demStep_m)};

    crg_data = crg_check(crg_data);

    ubeg = double(crg_data.head.ubeg);
    uvec = ubeg + (0:nu-1)' * uinc_m;

    [Ucrg, Vcrg] = ndgrid(uvec, vvec);
    puv = [Ucrg(:), Vcrg(:)];

    [pxy_grid, crg_data] = crg_eval_uv2xy(crg_data, puv);

    Xq_m = reshape(pxy_grid(:,1), nu, nv);
    Yq_m = reshape(pxy_grid(:,2), nu, nv);

    % ---------------------------------------------------------
    % 8) Sample the original DEM on the curved regular grid
    % ---------------------------------------------------------
    crg_z = interp2(xDEM_m, yDEM_m, Zsrc_m, ...
                    Xq_m, Yq_m, 'linear', NaN);

    nanCount = nnz(isnan(crg_z));
    if nanCount > 0
        % Show where the requested strip leaves the source DEM.
        plot(Xq_m(:,1)/toMeter,   Yq_m(:,1)/toMeter,   'r-', 'LineWidth',1.5);
        plot(Xq_m(:,end)/toMeter, Yq_m(:,end)/toMeter, 'r-', 'LineWidth',1.5);
        title(sprintf(['CRG strip exits DEM: %d samples are outside. ' ...
                       'Reduce half-width or move centerline.'], nanCount));
        drawnow;
        error(['Curved CRG contains %d NaN samples because the requested ' ...
               'strip leaves the source DEM.'], nanCount);
    end

    % On an explicitly closed loop, make the elevation seam exactly periodic.
    % The first and last CRG cross-sections represent the same physical seam;
    % averaging suppresses tiny interpolation/integration differences.
    if closeTrack
        seamZ = 0.5 * (crg_z(1,:) + crg_z(end,:));
        crg_z(1,:)   = seamZ;
        crg_z(end,:) = seamZ;
    end

    % ---------------------------------------------------------
    % 8) Preview the final curved strip on the DEM
    % ---------------------------------------------------------
    iCenter = nHalf + 1;

    % Check the ACTUAL evaluated CRG grid seam (not only header values).
    % For a closed track, the first and last centerline/border points must
    % coincide numerically.
    if closeTrack
        seamGapCenter = hypot(Xq_m(end,iCenter)-Xq_m(1,iCenter), ...
                              Yq_m(end,iCenter)-Yq_m(1,iCenter));
        seamGapRight  = hypot(Xq_m(end,1)-Xq_m(1,1), ...
                              Yq_m(end,1)-Yq_m(1,1));
        seamGapLeft   = hypot(Xq_m(end,end)-Xq_m(1,end), ...
                              Yq_m(end,end)-Yq_m(1,end));

        fprintf('  Evaluated center seam gap : %.12g m\n', seamGapCenter);
        fprintf('  Evaluated right seam gap  : %.12g m\n', seamGapRight);
        fprintf('  Evaluated left seam gap   : %.12g m\n', seamGapLeft);
    end

    plot(Xq_m(:,iCenter)/toMeter, Yq_m(:,iCenter)/toMeter, ...
         'c-', 'LineWidth',2.0);
    plot(Xq_m(:,1)/toMeter, Yq_m(:,1)/toMeter, ...
         'y-', 'LineWidth',1.4);
    plot(Xq_m(:,end)/toMeter, Yq_m(:,end)/toMeter, ...
         'y-', 'LineWidth',1.4);

    if closeTrack
        % Mark the first and last evaluated CRG center point separately.
        % When closure is correct, the magenta X sits exactly on the green O.
        plot(Xq_m(1,iCenter)/toMeter, Yq_m(1,iCenter)/toMeter, ...
             'go', 'MarkerSize',10, 'LineWidth',2.0);
        plot(Xq_m(end,iCenter)/toMeter, Yq_m(end,iCenter)/toMeter, ...
             'mx', 'MarkerSize',12, 'LineWidth',2.0);
    end

    % Draw a limited number of cross-sections so the figure remains readable.
    crossStep = max(1, floor(nu/25));
    for ii = 1:crossStep:nu
        plot(Xq_m(ii,[1 end])/toMeter, ...
             Yq_m(ii,[1 end])/toMeter, ...
             'w-', 'LineWidth',0.4);
    end

    title(sprintf('Curved CRG preview: %.1f m long x %.1f m wide', ...
        (nu-1)*uinc_m, vvec(end)-vvec(1)));
    drawnow;

    % ---------------------------------------------------------
    % 9) Replace temporary zero grid with sampled DEM elevations
    % ---------------------------------------------------------
    crg_data.z = single(crg_z);

    % Re-check after inserting the actual road surface.
    crg_data = crg_check(crg_data);
    crg_data = crg_single(crg_data);

    fprintf('\nFinal curved CRG matrix: %d x %d samples\n', ...
        size(crg_data.z,1), size(crg_data.z,2));
    fprintf('Final curved CRG dimensions: %.2f x %.2f m\n', ...
        (size(crg_data.z,1)-1)*uinc_m, ...
        (size(crg_data.z,2)-1)*vinc_m);

    % ---------------------------------------------------------
    % 10) Write OpenCRG file directly with the official writer
    % ---------------------------------------------------------
    CRGname = [PSname, '.crg'];
    ier = crg_write(crg_data, CRGname);

    if ier ~= 0
        warning('crg_write returned error code %d for %s.', ier, CRGname);
    else
        fprintf('%s is complete\n', CRGname);
    end

    clear xDEM_crg yDEM_crg fCRG xp yp pxy_user toMeter pxy_m ...
          xDEM_m yDEM_m Zsrc_m uinc_m ppxy refErr nu crgMinInc ...
          defaultHalfWidth halfWidth_user halfWidth_m vinc_m nHalf ...
          vvec nv crg_data ubeg uvec Ucrg Vcrg puv pxy_grid ...
          Xq_m Yq_m crg_z nanCount iCenter crossStep ii CRGname ier
end


%%  RDF Export
if max( ismember( Fext, 'rdf') )
    disp(' ')
    fprintf( 'Exporting RDF Data\n' )
    %  Create mesh or PLY
    %  convert to RDF

    % Colonne = X longitudinale; righe = Y trasversale. Si conserva
    % tutta la lunghezza X e si sceglie una striscia centrale in Y.
    % Questa larghezza RDF e' indipendente dal TrackWidth di coerenza.
    [nRowsDEM, nColsDEM] = size(Z);
    centerRow = round((nRowsDEM+1)/2);
    maxHalfCells = min(centerRow-1, nRowsDEM-centerRow);
    if nColsDEM < 2 || maxHalfCells < 1 || ~isfinite(sr) || sr <= 0
        error('DEM o samplerate non validi per RDF.');
    end
    defaultHalfCells = max(1, min(round(5*sr), maxHalfCells));
    defaultHalfWidth = defaultHalfCells/sr;
    fprintf('RDF: X longitudinale, Y trasversale.\n');
    fprintf('Semilarghezza massima: %.3f %s.\n', maxHalfCells/sr, units);
    while true
        widthText = strtrim(input(sprintf( ...
            'RDF half-width [%.3f %s; Invio = default] > ', ...
            defaultHalfWidth, units), 's'));
        if isempty(widthText)
            halfWidthRequested = defaultHalfWidth;
        else
            halfWidthRequested = str2double(strrep(widthText, ',', '.'));
        end
        if ~isscalar(halfWidthRequested) || ...
                ~isfinite(halfWidthRequested) || halfWidthRequested <= 0
            fprintf('Inserisci una semilarghezza positiva.\n');
            continue;
        end
        halfCells = round(halfWidthRequested*sr);
        if halfCells < 1 || halfCells > maxHalfCells
            fprintf('Scegli una semilarghezza tra %.3f e %.3f %s.\n', ...
                1/sr, maxHalfCells/sr, units);
            continue;
        end
        break;
    end

    rdf_z = Z(centerRow-halfCells:centerRow+halfCells, :);
    if any(~isfinite(rdf_z(:)))
        error('La striscia RDF contiene quote NaN o Inf.');
    end
    [a,b] = size(rdf_z); % a = righe Y, b = colonne X
    xvec = (0:b-1)/sr;
    yvec = ((0:a-1)-(a-1)/2)/sr;
    [X,Y] = meshgrid(xvec,yvec);
    x = X(:);
    y = Y(:);
    z = rdf_z(:); % come prima: quote relative alla media del DEM

    % Due triangoli per cella; nodi in ordine antiorario visti da +Z.
    [rows,cols] = ndgrid(1:a-1,1:b-1);
    n00 = rows(:)+(cols(:)-1)*a;
    DT = [n00, n00+a, n00+a+1; ...
          n00, n00+a+1, n00+1];

    %  RDF matrix
    M2 = [ x y z ];
    
    if strcmp( units, 'ft' )
        M2 = M2 *12*2.54/100;  %  convert feet to meters
    end

    Length = max(M2(:,1));
    
    M2 = M2 .* 1000;  % meters to millimeters, ADAMS preference
    
    %  Create RDF file  ======================================================

    outDir = fullfile(pwd,'AVT_Out');
    if ~exist(outDir,'dir')
        [ok,msg] = mkdir(outDir);
        if ~ok, error('Impossibile creare AVT_Out: %s', msg); end
    end
    RDFname = fullfile(outDir, [PSname,'.rdf']);
    fprintf('Saving RDF file...\n');
    tic
    [f1,msg] = fopen(RDFname,'w');
    if f1 < 0, error('Impossibile aprire RDF: %s',msg); end
    
    %  find number of digits needed
    % str = num2str( round(Length * 1000) );
    % dig = numel( str );
    
    % Print Header  ===========================================================
    
    fprintf( f1, '$---------------------------------------------------------------------MDI_HEADER\r\n' );
    fprintf( f1, '[MDI_HEADER]\r\n' );
    fprintf( f1, 'FILE_TYPE  =  ''rdf''\r\n' );
    fprintf( f1, 'FILE_VERSION  =  5.00\r\n' );
    fprintf( f1, 'FILE_FORMAT  =  ''ASCII''\r\n' );
    fprintf( f1, '(COMMENTS)\r\n' );
    fprintf( f1, '(comment_string)\r\n' );
    fprintf( f1, '''Course Name %s''\r\n', PSname );
    fprintf( f1, '''Left Length = %.3f meters''\r\n', Length );
    fprintf( f1, '''Right length = %.3f meters''\r\n', Length );
    fprintf( f1, '$--------------------------------------------------------------------------units\r\n' );
    fprintf( f1, '[UNITS]\r\n' );
    fprintf( f1, 'LENGTH             = ''millimeter''\r\n' );
    fprintf( f1, 'FORCE              = ''newton''\r\n' );
    fprintf( f1, 'ANGLE              = ''radians''\r\n' );
    fprintf( f1, 'MASS               = ''kg''\r\n' );
    fprintf( f1, 'TIME               = ''sec''\r\n' );
    fprintf( f1, '$---------------------------------------------------------------------definition\r\n' );
    fprintf( f1, '[MODEL]\r\n' );
    fprintf( f1, 'METHOD	=	''3D''\r\n' );
    fprintf( f1, '$-------------------------------------------------------------------------offset\r\n' );
    fprintf( f1, '[OFFSET]\r\n' );
    fprintf( f1, 'X                  =  0.0\r\n' );
    fprintf( f1, 'y                  =  0.0\r\n' );
    fprintf( f1, 'Z                  =  0.0\r\n' );
    fprintf( f1, '$--------------------------------------------------------------------------nodes\r\n' );
    fprintf( f1, '[NODES]\r\n' );
    fprintf( f1, 'NUMBER_OF_NODES =                  %d\r\n', numel( M2(:,1) ) );
    fprintf( f1, '{    #        X         Y         Z     }\r\n' );
    
    fclose( f1 );
    
    %  Print Nodes  ===========================================================
    
    % tmp = 5000;
    tmp = 1:size(M2,1);
    M2 = [ tmp' round( M2 ) ];
    
    writematrix(M2, RDFname, 'FileType', 'text', 'WriteMode', 'append', 'Delimiter', '\t' )
    
    
    %  Transition header  =====================================================
    
    [f1,msg] = fopen(RDFname,'a');
    if f1 < 0, error('Impossibile riaprire RDF: %s',msg); end

    fprintf( f1, '$----------------------------------------------------------------------elements\r\n' );
    fprintf( f1, '[ELEMENTS]\r\n' );
    fprintf( f1, 'NUMBER_OF_ELEMENTS =               %d\r\n', numel(DT(:,1)) );
    fprintf( f1, '{  Node 1   Node 2    Node 3      mu    }\r\n' );
    
    fclose( f1 );
    
    % Print faces  ============================================================
    
    mu = ones(size(DT,1), 1) * 0.8;
    DT = [ DT mu ];
    
    writematrix(DT, RDFname, 'FileType', 'text', 'WriteMode', 'append', 'Delimiter', '\t' )
    
    fprintf('RDF mesh: %d x %d nodes; %d triangles.\n', b,a,size(DT,1));
    fprintf('RDF size: %.3f x %.3f m; requested half-width: %.3f %s.\n', ...
        (max(M2(:,2))-min(M2(:,2)))/1000, ...
        (max(M2(:,3))-min(M2(:,3)))/1000, ...
        halfWidthRequested, units);
    fprintf('%s is complete\n', RDFname);

    clear a b rdf_z X Y x y z DT M2 Length RDFname tmp mu ...
          nRowsDEM nColsDEM centerRow maxHalfCells defaultHalfCells ...
          defaultHalfWidth widthText halfWidthRequested halfCells ...
          rows cols n00 outDir ok msg f1 
end


%%  PLY Export
%% Check and initialize PLY path dynamically
if max(ismember(Fext, 'ply'))
    if ~exist('plywrite.m', 'file')
        try
            disp(' ')
            disp('Searching for PLY code path for initialization...')
            CP = pwd;

            % Search recursively for plywrite.m starting from current directory
            List = dir(fullfile(CP, '**', 'plywrite.m'));

            % Walk up parent directory tree if not found in pwd
            while isempty(List)
                CPnew = fileparts(CP);
                if strcmp(CPnew, CP)
                    error('PLY directory containing plywrite.m not found.');
                end
                CP = CPnew;
                List = dir(fullfile(CP, '**', 'plywrite.m'));
            end

            % Add the directory containing plywrite.m to MATLAB path
            plyFolder = List(1).folder;
            addpath(plyFolder);
            disp('PLY code found and added to MATLAB path.')

        catch ME
            disp(' ')
            disp('"Plywrite" code not found - download at:')
            url = 'https://www.mathworks.com/matlabcentral/fileexchange/55171-plywrite';
            fprintf(1, '<a href="matlab: web(''%s'') ">%s</a>\n', url, url);
            fprintf('PLY initialization failed: %s\n', ME.message);

            % Safely remove 'ply' from requested export list to prevent crashes
            ind = ismember(Fext, 'ply');
            Fext(ind) = [];
        end
    end
end


if max( ismember( Fext, 'ply') )
    disp(' ')
    fprintf( 'Exporting PLY Data\n' )
    %  Create mesh or PLY
    %  convert to RDF

    str = input( 'Create Full Surface or Single Lane PLY? [S/L] > ', 's' );

    if contains( upper(str), 'L' )
        %  trim to single lane
        %  Lane width = 2 X Track Width
        disp( 'Center lane 2X Trackwidth is being exported to PLY.' )
        disp( 'Alter "PLY Export" code if a different lane is desired.')

        TW_ct = round( TW * sr );
        [ a, b ] = size( Z );

        %  All rows, center lane columns
        ply_z = Z(:, (b/2-TW_ct):(b/2+TW_ct));

        %  All columns, center lane rows
        % ply_z = Z( (a/2-TW_ct):(a/2+TW_ct), :)';
    else
        disp( 'Full Surface PLY is being created...')
        %  Full surface
        ply_z = Z;
    end
    [a,b] = size(ply_z);   % a = Y rows, b = X columns

    xvec = (0:b-1)/sr;
    yvec = ((0:a-1) - (a-1)/2)/sr;
    [X,Y] = meshgrid(xvec, yvec);

    x = X(:);
    y = Y(:);
    z = ply_z(:);

    DT = delaunay( x, y );
    % trimesh( DT, X', Y', ply_z )

    plywrite( ['AVT_Out\', PSname,'.ply'], DT, [x, y, z] );
    fprintf( '%s is complete\n', [PSname,'.ply'] )

    clear TW_ct a b ply_z X Y x y z DT 
end

disp(' ')
disp( 'TerrainReport execution is complete' )
