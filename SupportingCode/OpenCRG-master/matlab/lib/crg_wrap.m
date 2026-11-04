function [data] = crg_wrap(data)
%CRG_WRAP Wrap heading angles to +/- pi range.
%   [DATA] = CRG_WRAP(DATA) wraps OpenCRG heading angle data in road parameters
%   and road data to a +/- pi range.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    struct array with updated heading angles.
%
%   Examples:
%   data = crg_wrap(data);
%       Wraps heading angles in road parameters and road data to a +/- pi range.
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

%% convert core vectors and arrays to single type

% head data
if isfield(data.head, 'pbeg')
    data.head.pbeg = atan2(sin(data.head.pbeg), cos(data.head.pbeg));
end
if isfield(data.head, 'pend')
    data.head.pend = atan2(sin(data.head.pend), cos(data.head.pend));
end

% core reference line data
if isfield(data, 'p')
    if isa(data.p, 'double')
        data.p = atan2(sin(data.p), cos(data.p));
    else
        data.p = single(atan2(sin(double(data.p)), cos(double(data.p))));
    end
end

end
