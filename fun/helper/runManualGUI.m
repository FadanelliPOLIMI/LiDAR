function [totalShiftEast_m, totalShiftNorth_m] = runManualGUI(ctx)
% RUNMANUALGUI Launches the interactive shift control panel
global totalShiftEast_m totalShiftNorth_m spanX_meters spanY_meters imgWidth imgHeight lidarCompositeRGB satImg figRef stepPopup sessionActive;

spanX_meters = ctx.spanX_meters;
spanY_meters = ctx.spanY_meters;
imgWidth = ctx.imgWidth;
imgHeight = ctx.imgHeight;
lidarCompositeRGB = ctx.lidarCompositeRGB;
satImg = ctx.satImg;

totalShiftEast_m = 0;
totalShiftNorth_m = 0;

disp('>>> [Manual Mode] Opening interactive control panel...');

% Main map display figure (top)
figRef = figure('Name', 'Interactive LiDAR on Satellite Overlay Control', ...
    'Position', [100, 200, 900, 700]);

% Shift Control Panel figure positioned cleanly DIRECTLY BELOW the main figure
figCtrl = figure('Name', 'Shift Control Panel', ...
    'Position', [100, 80, 300, 320], ...
    'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off');

uicontrol('Style', 'text', 'Position', [20, 265, 260, 30], 'String', 'Select Shift Step Size:', 'FontSize', 10, 'FontWeight', 'bold');
stepPopup = uicontrol('Style', 'popupmenu', 'Position', [20, 235, 260, 30], 'String', {'0.1 Meters', '0.5 Meters', '1 Meter', '5 Meters', '10 Meters'}, 'Value', 3);

uicontrol('Style', 'pushbutton', 'String', '▲ NORTH', 'Position', [110, 165, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(0, getStepVal()));
uicontrol('Style', 'pushbutton', 'String', '▼ SOUTH', 'Position', [110, 75, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(0, -getStepVal()));
uicontrol('Style', 'pushbutton', 'String', '◄ WEST', 'Position', [20, 120, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(-getStepVal(), 0));
uicontrol('Style', 'pushbutton', 'String', '► EAST', 'Position', [200, 120, 80, 40], 'FontSize', 10, 'Callback', @(~,~) moveCloud(getStepVal(), 0));
uicontrol('Style', 'pushbutton', 'String', '✔ SAVE AND CLOSE', 'Position', [20, 15, 260, 45], 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.8 0.2], 'Callback', @(~,~) closePanel());

sessionActive = true;
updatePlot(); 

while sessionActive && ishandle(figRef) && ishandle(figCtrl)
    pause(0.1);
end

try close(figCtrl); catch; end
try close(figRef); catch; end
end

function stepVal = getStepVal()
global stepPopup;
vals = [0.1, 0.5, 1, 5, 10];
stepVal = vals(stepPopup.Value);
end

function moveCloud(dEast, dNorth)
global totalShiftEast_m totalShiftNorth_m;
totalShiftEast_m = totalShiftEast_m + dEast;
totalShiftNorth_m = totalShiftNorth_m + dNorth;
updatePlot();
end

function updatePlot()
global totalShiftEast_m totalShiftNorth_m spanX_meters spanY_meters imgWidth imgHeight lidarCompositeRGB satImg figRef;
pxShift = round((totalShiftEast_m / spanX_meters) * imgWidth);
pyShift = round(-(totalShiftNorth_m / spanY_meters) * imgHeight);
shiftedLiDAR = imtranslate(lidarCompositeRGB, [pxShift, pyShift], 'FillValues', 0);
if ishandle(figRef)
    set(0, 'CurrentFigure', figRef);
    clf(figRef);
    imshowpair(satImg, shiftedLiDAR, 'blend');
    title(sprintf('MANUAL INTERACTIVE OVERLAY\nShift: E = %.2f m | N = %.2f m', totalShiftEast_m, totalShiftNorth_m));
    drawnow;
end
end

function closePanel()
global sessionActive;
sessionActive = false;
disp('>>> [Confirmed] Interactive alignment saved.');
end