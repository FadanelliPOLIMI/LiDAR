function [llh dat] = map_plocal2global(enh, dat)
% MAP_LOCAL2GLOBL Backward projection: projected local to global.
%   [LLH DAT] = MAP_LOCAL2GLOBL(ENH, DAT) converts points from
%   local map coordinates to global geodetic coordinates by backward projection
%   on a local ellipsoid and datum transformation from a local to global
%   ellipsoid.
%
%   Inputs:
%   ENH     (n, 3) array of points in PMAP system
%   DAT     opt struct array with
%       .lell   ELLI struct of local geodetic datum
%       .gell   ELLI struct of global geodetic datum
%       .tran   TRAN struct of datum transformation
%       .proj   PROJ struct of map projection
%
%   Outputs:
%   LLH     (n, 3) array of points in WGS84 GEOD system
%   DAT     struct array with
%       .lell   ELLI struct of local geodetic datum
%       .gell   ELLI struct of global geodetic datum
%       .tran   TRAN struct of datum transformation
%       .proj   PROJ struct of map projection
%
%   Examples:
%   llh = map_plocal2global(enh, dat)
%       Converts points from map coordinates to global geodetic coordinates.
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

% DAT
if nargin < 2, dat = []; end

dat = map_check(dat);

gell = dat.gell;
lell = dat.lell;
tran = dat.tran;
proj = dat.proj;

%ENH
enh_local = enh;

%% process

[llh_local lell proj] = map_pmap2geod_tm(enh_local, lell, proj);

if isequal(gell, lell) && strcmp(tran.nm, 'NOP')
    llh_globl = llh_local;
else
    [xyz_local lell] = map_geod2ecef(llh_local, lell);
    [xyz_globl tran] = map_ecef2ecef(xyz_local, tran, 'B');
    [llh_globl gell] = map_ecef2geod(xyz_globl, gell);
end

%% prepare outputs

llh = llh_globl;

dat = struct;
dat.gell = gell;
dat.lell = lell;
dat.tran = tran;
dat.proj = proj;

end