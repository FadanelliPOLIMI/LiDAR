function [enh ell pro] = map_geod2pmap(llh, ell, pro)
% MAP_GEOD2PMAP Forward map projection.
%   [ENH ELL PRO] = MAP_GEOD2PMAP_TM(LLH, ELL, PRO) converts points from
%   geodetic coordinates to map coordinates using a forward projection.
%
%   Inputs:
%   LLH     (n, 3) array of points in GEOD system
%   ELL     optional ELLI struct array
%   PRO     opt. PROJ struct array
%
%   Outputs:
%   ENH     (n, 3) array of points in PMAP system
%   ELL     ELLI struct array
%   PRO     PROJ struct array
%
%   Examples:
%   enh = map_geod2pmap_tm(llh, ell, pro)
%       Converts points from geodetic to map coordinates.
%
%   See also MAP_INTRO.

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

%% check and complement inputs

% PRO
if nargin < 3
    pro = [];
end
pro = map_check_proj(pro);

% ELL
if nargin < 2
    ell = [];
end
ell = map_check_elli(ell);

%% apply projection

if ~isempty(nm)
    a = regexp([upper(nm) '_'], '_', 'split');
    nm = a{1};
end

if ~isempty(nm)
    switch nm
        case 'GK3' % Gauss-Krueger 3deg zones
            [enh ell pro] = map_geod2pmap_tm(llh, ell, pro);
        case 'GK6' % Gauss-Krueger 6deg zones
            [enh ell pro] = map_geod2pmap_tm(llh, ell, pro);
        case 'UTM' % Universal Transverse Mercator
            [enh ell pro] = map_geod2pmap_tm(llh, ell, pro);
        otherwise
            error('MAP:checkError', 'unknown projection name %s', nm)
    end
end



