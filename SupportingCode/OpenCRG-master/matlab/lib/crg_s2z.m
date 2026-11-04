function [data] = crg_s2z(data, rz)
% CRG_S2Z Apply slope to OpenCRG data.
%   CRG_S2Z(DATA, RZ) applies new slope to an OpenCRG struct. Existing
%   slope data is merged into the road data.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO
%   RZ      (optional) refline elevation profile (default: (zbeg+zend)/2)
%           length(rz) == 1: no slope
%           length(rz) == 2: constant slope
%           length(rz) == nu: variable slope
%
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   crg = crg_s2z(crg)
%       Merges existing slope information into the elevation grid.
%   crg = crg_s2z(crg, [crg.head.zbeg crg.head.zend])
%       Apply constant slope to elevation grid
%   crg = crg_s2z(crg, rz)
%       Apply slope defined by reference line elevation profile.

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

[nu] = size(data.z,1);

%% handle optional arguments

if nargin < 2
    rz = (data.head.zbeg + data.head.zend) / 2;
end

if length(rz)~=1 && length(rz)~=2 && length(rz)~=nu
    error('CRG:s2zError', 'illegal size of rz')
end

if length(rz) ~= nu
    rz = linspace(rz(1), rz(end), nu);
end

%% get old rz

if isfield(data, 'rz')
    rzold = data.rz;
else
    rzold = linspace(data.head.zbeg, data.head.zend, nu);
end

%% set new slope

s = diff(rz) / data.head.uinc;

data.head.sbeg = s(1);
data.head.send = s(end);

data.s = single(s);

data.head.zbeg = rz(1);
data.head.zend = rz(end);

%% adjust altitude

if isfield(data.head, 'abeg')
    data.head.abeg = data.head.abeg + rz(1) - rzold(1);
    data.head.aend = data.head.abeg + (rz(end)-rz(1));
end

%% apply reference line elevation profile

for i = 1:nu
    data.z(i,:) = single(double(data.z(i,:)) + (rzold(i) - rz(i)));
end

%% check data

data = crg_check(data);
if ~isfield(data, 'ok')
    error('CRG:checkError', 'check of DATA was not completely successful')
end

end
