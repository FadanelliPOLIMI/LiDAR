function [data] = crg_single(data)
%CRG_SINGLE Convert OpenCRG road data to type single.
%   [DATA] = CRG_SINGLE(DATA) converts OpenCRG road data to type
%   single.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    input DATA with road data converted to type single.
%
%   Examples:
%   data = crg_single(data);
%       Converts CRG road data to type single.
%
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


%% convert core vectors and arrays to type single

% core elevation grid data
if isfield(data, 'z'), data.z = single(data.z); end
if isfield(data, 'v'), data.v = single(data.v); end
if isfield(data, 'b'), data.b = single(data.b); end

% core reference line data
if isfield(data, 'u'), data.u = single(data.u); end
if isfield(data, 'p'), data.p = single(data.p); end
if isfield(data, 's'), data.s = single(data.s); end

end
