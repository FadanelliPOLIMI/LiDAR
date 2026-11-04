function [data] = crg_plot_elgrid_uvz_map(data, iu, iv)
% CRG_PLOT_ELGRID_UVZ_MAP Plot z-values as uncurved orthographic image.
%   DATA = CRG_PLOT_ELGRID_UVZ_MAP(DATA, IU, IV) plots z-values as an
%   orthographic image over an uncurved grid in the current axes object.
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
%   data = crg_plot_elgrid_uvz_map(data)
%       Plots all z-values.
%   data = crg_plot_elgrid_uvz_map(data, [1000 2000], [10 30])
%       Plots z-values in the selected area of the grid.
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
    error('CRG:plotError', 'illegal IU index values iu=[%d %d] with nu=%d', iu(1), iu(2), nu);
end

if nargin < 3
    iv =  [1 nv];
end
if iv(1)<1 || iv(1) >= iv(2) || iv(2) > nv
    error('CRG:plotError', 'illegal IV index values iv=[%d %d] with nv=%d', iv(1), iv(2), nv);
end

%% generate auxiliary data

nuiu = iu(2) - iu(1) + 1;
nviv = iv(2) - iv(1) + 1;

u = zeros(1, nuiu);
for i = 1:nuiu
    u(i) = data.head.ubeg + (i-1+iu(1)-1)*data.head.uinc;
end

if isfield(data.head, 'vinc')
    v = zeros(1, nviv);
    for i=1:nviv
        v(i) = data.head.vmin + (i-1+iv(1)-1)*data.head.vinc;
    end
else
    v = double(data.v(iv(1):iv(2)));
end

z = double(data.z(iu(1):iu(2),iv(1):iv(2)));

%% plot elevation grid UVZ orthographic map

data = crg_surf(data, u, v, z');

xlabel('U [m]')
ylabel('V [m]')
zlabel('Z [m]')
title('CRG elevation grid UVZ map (in uncurved UV grid)')

view(2)

end
