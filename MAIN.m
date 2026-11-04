%% DATA PROCESSIN2G & TACTICAL TRANSMISSION PIPELINE %%
clear; clc;
fprintf('\n--------------------------------------------\n');
fprintf('BOOTSTRAPPING ENVIRONMENT & FOLDERS\n');
fprintf('--------------------------------------------\n');

% 1. Automatically create required directories if they do not exist
requiredDirs = {'fun', 'files', fullfile('files', 'configs')};
for i = 1:length(requiredDirs)
    if ~exist(requiredDirs{i}, 'dir')
        mkdir(requiredDirs{i});
        fprintf('  -> Created missing directory: %s\n', requiredDirs{i});
    else
        fprintf('  -> Verified directory exists: %s\n', requiredDirs{i});
    end
end

% 2. Add subdirectories to MATLAB search path
addpath(genpath('fun'));
addpath(genpath('files'));

% 3. Initialize or load configuration safely
config = initPipelineConfig();


fprintf('  -> Operating Mode: %s\n', upper(config.mode));
fprintf('--------------------------------------------\n');
%% 0. DATA SIMULATION
% 0A. PIPELINE RUN WITH CHUNKED .LAZ DATA
%[batchFiles, config] = step0a_runPipeline(config, 'FileName', 'Alta Frequenza 5 cm.las', 'Force', true);
[~, config] = step0a_runPipeline(config,'Force', true);
%ENDS WITH DATA CHUNKED FOR NEXT ANALYSIS
% 0B. REASSEMBLE DATA
step0b_reconstructTacticalBursts(config);

%% 1. LAS & WKT READING
fprintf('\n--------------------------------------------\n');
fprintf('STAGE 1: POINTCLOUD READING\n');
fprintf('--------------------------------------------\n');
%[ptCloudRaw, wkt, ascFile, prjFile] = step1_loadLAS(config);
[ptCloudRaw, wkt, ascFile, prjFile] = step1_loadLAS(config, 'DataType', 'Reconstructed');
%% 2. DYNAMIC PRE-FILTER: DENOISING STATISTICO (SOR) - VOXEL ADAPTATIVE DOWNSAMPLING - PCA
fprintf('\n--------------------------------------------\n');
fprintf('STAGE 2: UNIFIED PREPROCESSING & FEEDBACK GATE\n');
fprintf('--------------------------------------------\n');
% Run unified Stage 2 preprocessing
[ptCloudFiltered,config, stage2Metrics] = step2_preFilter(ptCloudRaw, config);

%% 3. AUTOMATIC / INTERACTIVE ROAD EXTRACTION
fprintf('\n--------------------------------------------\n');
fprintf('STAGE 3: ROAD EXTRACTION & GEOMETRIC ALIGNMENT\n');
fprintf('--------------------------------------------\n');
% Run unified Stage 3
[ptCloudExtracted, ptCloudAligned, config, stage3Metrics] = step3_extractROI(ptCloudFiltered, config);

%% 4. MASTER REFERENCE, CONDITIONAL INDEPENDENT OPTIMIZATION & BENCHMARKING
fprintf('\n------------------------------------------------------------\n');
fprintf('>>> STAGE 4: ROAD CLASSIFICATION & BENCHMARKING\n');
fprintf('------------------------------------------------------------\n');
[ptCloudRoad, winningMethod, benchmarkMetrics, bestHeuristicParams, rfModel] = step4_classFilters(ptCloudExtracted,ptCloudAligned, ptCloudFiltered, config);

%% % 5. DOWNSTREAM ROAD EXTRACTION & EXPORT
fprintf('\n------------------------------------------------------------\n');
fprintf('>>> STAGE 5: ROAD SURFACE REFINEMENT & RASTERIZATION\n');
fprintf('------------------------------------------------------------\n');
ptCloudCleanRoad=step5_roadCutter(ptCloudRoad,config);

%% % 6: SURFACE RECONSTRUCTION & HOLE INPAINTING
fprintf('\n------------------------------------------------------------\n');
fprintf('>>> STAGE 6: ROAD SURFACE RECONSTRUCTION\n');
fprintf('------------------------------------------------------------\n');
[ptCloudFinalMerged, ptCloudSynthetic] = step6_fillRoadHoles(ptCloudCleanRoad, config);

%% % 7: AREA OF INTEREST (ROI) SELECTION 
fprintf('\n-------------------------------------------------------------n');
fprintf('>>> STAGE 7: ROI SELECTION\n');
fprintf('------------------------------------------------------------\n');
% Optional: Pre-define bounding box limits in config if running headless
% config.roiLimits = [20, 80, -3, 3]; % [xmin, xmax, ymin, ymax]
ptCloudFinalROI = step7_selectROI(ptCloudFinalMerged, config);

%% % 8: ROAD PROFILE RASTERIZATION FOR PROCESSING IN TERRAIN_REPORT
fprintf('\n-------------------------------------------------------------n');
fprintf('>>> STAGE 8: SMOOTHING AND RASTER\n');
fprintf('------------------------------------------------------------\n');
% Execute the unified processing and export function
Zterrain = step8_exportTerrain(ptCloudFinalROI, config);
%plotRaster(Zterrain,config)

%% % 9: RUN TERRAIN_REPORT
fprintf('\n------------------------------------------------------------\n');
fprintf('>>> STAGE 9: RUN TERRAIN REPORT & EXPORT\n');
fprintf('------------------------------------------------------------\n');

% 1. Ensure output directory exists
if ~exist('AVT_Out', 'dir')
    mkdir('AVT_Out');
end

% 1. Auto-crop outer NaN borders so we only analyze valid road data
validRows = find(~all(isnan(Zterrain), 2));
validCols = find(~all(isnan(Zterrain), 1));
Z_cropped = Zterrain(min(validRows):max(validRows), min(validCols):max(validCols));

% 2. Safe Local Fallback for Analysis Only:
% If there are still micro-NaNs at the absolute jagged edges of the crop, 
% fill them locally using nearest neighbor so pwelch doesn't crash, 
% while leaving your master Zterrain untouched for simulations.
Z_analysis = fillmissing(Z_cropped, 'nearest', 2);
Z_analysis = fillmissing(Z_analysis, 'nearest', 1);

% 3. Configure GIS structure using the clean analysis matrix
GIS = struct();
GIS.mode = 'real';
GIS.units = 'm';
GIS.Zmatrix = double(Z_analysis); 
GIS.TrackWidth = 2.0;               % Vehicle track width [m]
GIS.samplerate = 1 / config.voxelSize; 
GIS.CutOff = 20.0;                  % Wavelength upper cutoff [m]
GIS.wkt = wkt;
GIS.Fext = {'mat','las', 'ply', 'rdf', 'crg','xyz','tif'}; 

fprintf('  -> Executing TerrainReport_rectangular export routines...\n');
%roadHullCoeffs = 
TerrainReport_rectangular_old(GIS);

fprintf('\n============================================\n');
fprintf('  SUCCESS: Simulation-ready files generated!\n');
fprintf('  Extracted X-Exponent: %.2f\n', roadHullCoeffs.X_Exponent);
fprintf('============================================\n');

%% 10. GESTIONE INCERTEZZA E MONTE CARLO (La nostra pipeline Master)
[optimalRoadGrid, reportTable, stochasticVariations] = run_MasterBayesianRoadPipeline(ptCloudRoad, 0.1);

%% 22. prova simscape
% Specify the width of the road from the reference line.
dataFolder = tempdir;
lidarDataFileName = "DigitransData.zip"; 
url = "https://ssd.mathworks.com/supportfiles/driving/data/" + lidarDataFileName;
filePath = fullfile(dataFolder,lidarDataFileName); 
if ~isfile(filePath)
    websave(filePath,url,Timeout=100)
end
unzip(filePath,dataFolder)

sensorData = load(fullfile(dataFolder,"sensorData.mat"));

traj = sensorData.traj;
idx = 5 : traj.NumSamples - 4;

traj = scenariobuilder.Trajectory(traj.Timestamps(idx), traj.Position(idx,:));
NameValueArgs.Origin = [];
ptCld = ptCloudFinalROI;
if isempty(NameValueArgs.Origin)
    centerPt = [mean(ptCld.XLimits), mean(ptCld.YLimits), mean(ptCld.ZLimits)];
else
    centerPt = NameValueArgs.Origin;
end
transformedPointCloud = pctransform(ptCld, rigidtform3d([0,0,0],-centerPt));

ax = pcshow(transformedPointCloud.Location,ViewPlane="XY");
hold(ax,"on")
plot3(ax,traj.Position(:,1),traj.Position(:,2),traj.Position(:,3),LineWidth=5)
xlabel(ax,"X-Axis")
ylabel(ax,"Y-Axis")
hold(ax,"off")

roadWidthFromRefLine = 4;

% Specify a resolution of 6 cm in the longitudinal and 4.5 cm in the lateral direction.
gridResolution = [0.06 0.045];

% Smooth the road reference line to remove any jitters.
%traj = smooth(traj,Method="sgolay",Degree=3,SmoothingFactor=0.2);

% Generate the road surface.
rsObj = roadSurface(transformedPointCloud,traj,roadWidthFromRefLine,gridResolution,NormalizeElevation=false,LocalOrigin=[0,0,0],ShowProgress=false);
show(rsObj)
% Specify t;he name of the OpenCRG file.
filename = "digitransRoad.crg";

% Export the road surface to an ASAM OpenCRG file.
exportToASAMOpenCRG(rsObj,filename)

%%
pathToSimscapeVehicleTemplatesFile = fullfile('/home/simpa-nelli/Documents/MATLAB/PROJECTS/Examples/R2026b/Simscape-Vehicle-Templates-master.zip');

% Specify the full path to the CRG File.
crgFile = fullfile(pwd,filename);

helperSetupSimscapeVehicleTemplates(pathToSimscapeVehicleTemplatesFile,crgFile)

%%
[Maneuver,Init] = helperConfigureSimulation(traj,Init,MDatabase);

sm = simulation("sm_car");
sm = setModelParameter(sm,StopTime="30");
initialize(sm)

start(sm)
%%