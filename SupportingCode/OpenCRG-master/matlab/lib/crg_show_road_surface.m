function [data] = crg_show_road_surface(data, iu, iv)
% CRG_SHOW_ELGRID_SURFACE Visualize the road surface.
%   DATA = CRG_SHOW_ELGRID_SURFACE(DATA, IU, IV) visualizes the road surface
%   via orthographic images and 3-dimensional surface plots. The plots can be
%   limited to a selected area of the grid.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO
%   IU      U index for plot selection (default: full CRG)
%           IU(1): longitudinal start index
%           IU(2): longitudinal end index
%   IV      V index for plot selection (default: full CRG)
%           IV(1): lateral start index
%           IV(2): lateral end index
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   data = crg_show_road_cuts_surface(data)
%       Visualizes the full CRG.
%   data = crg_show_road_cuts_surface(data, [1000 2000], [10 20])
%       Visualizes the selected part of the CRG.
%   See also CRG_INTRO.

% *****************************************************************
% See the NOTICE file distributed with this work regarding copyright ownership.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%    https://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%
% More Information on ASAM OpenCRG can be found here:
% https://www.asam.net/standards/detail/opencrg/
%
% *****************************************************************

%% first check, fix and complement DATA

%% check if already successfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of DATA was not completely successful')
    end
end

%% evaluate DATA.z size

[nu nv] = size(data.z);

%% check and complement optional arguments

if nargin < 2
    iu =  [1 nu];
end
if iu(1)<1 || iu(1) >= iu(2) || iu(2) > nu
    error('CRG:showError', 'illegal IU index values iu=[%d %d] with nu=%d', iu(1), iu(2), nu);
end

if nargin < 3
    iv =  [1 nv];
end
if iv(1)<1 || iv(1) >= iv(2) || iv(2) > nv
    error('CRG:showError', 'illegal IV index values iv=[%d %d] with nv=%d', iv(1), iv(2), nv);
end

%% define figure frame

if ~isfield(data, 'fopt') || ~isfield(data.fopt, 'tit')
    data.fopt.tit = 'CRG road surface';
    data          = crg_figure(data);
    data.fopt     = rmfield(data.fopt, 'tit');
else
    data = crg_figure(data);
end

%% reference line XY overview map

subplot(2,2,1)
data = crg_plot_refline_xy_overview_map(data, iu);
set(    gca             , 'ButtonDownFcn','copy_ax2fig')
set(get(gca, 'Children'), 'ButtonDownFcn','copy_ax2fig')

%% elevation grid XYZ perspective map

subplot(2,2,2)
data = crg_plot_road_xyz_map(data, iu, iv);
set(    gca             , 'ButtonDownFcn','copy_ax2fig')
set(get(gca, 'Children'), 'ButtonDownFcn','copy_ax2fig')

%% elevation grid UVZ orthographic map

subplot(2,2,[3 4])
data = crg_plot_road_uvz_map(data, iu, iv);
set(    gca             , 'ButtonDownFcn','copy_ax2fig')
set(get(gca, 'Children'), 'ButtonDownFcn','copy_ax2fig')

end
