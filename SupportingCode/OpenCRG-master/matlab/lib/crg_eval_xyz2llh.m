function [pllh, data] = crg_eval_xyz2llh(data, pxyz)
%CRG_EVAL_XYZ2LLH Transform point in xyz to llh.
%   [PLLH, DATA] = CRG_EVAL_XYZ2LLH(DATA, PXYZ) transforms points given in
%   xyz-coordinates to llh-coordinates.
%
%   inputs:
%       DATA    struct array as defined in CRG_INTRO.
%       PXYZ    (np, 3) array of points in xyz-system (CRG local)
%
%   outputs:
%       PLLH    (np, 3) array of points in llh-system (GEOD global)
%       DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   [pllh, data] = crg_eval_xyz2llh(data, pxyz) transforms pxyz points
%   given in local CRG coordinate system to pllh points given in GEOD
%   global coordinate system.
%
%   See also CRG_INTRO, MAP_INTRO, CRG_EVAL_LLH2XYZ.

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

%% check if MPRO is available

if ~isfield(data, 'mpro')
    error('CRG:evalError', 'no map projection data available')
end

%% perform transformation CRG local -> CRG global

[penh, data] = crg_eval_xyz2enh(data, pxyz);

%% perform transformation CRG global -> GEOD global

[pllh, data.mpro] = map_plocal2global(penh, data.mpro);

end
