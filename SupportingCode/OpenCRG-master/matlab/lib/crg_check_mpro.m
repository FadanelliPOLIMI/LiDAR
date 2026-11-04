function [data] = crg_check_mpro(data)
% CRG_CHECK_MPRO Check OpenCRG map projection data.
%   [DATA] = CRG_CHECK_MPRO(DATA) checks OpenCRG map projection data for
%   consistent definitions and values.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    input DATA with checked map projection data.
%
%   Examples:
%   data = crg_check_mpro(data)
%       Checks CRG map projection data.
%
%   See also CRG_CHECK, CRG_INTRO.

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

%% remove ok flag, initialize error/warning counter

if isfield(data, 'ok')
    data = rmfield(data, 'ok');
end
ierr = 0;

%% check for mpro field

if isfield(data, 'mpro')
    try
        data.mpro = map_check(data.mpro);
    catch exception
        getReport(exception)
        warning('CRG:checkWarning', 'inconsistent or illegal map projection data')
        ierr = 1;
    end
end


%% set ok-flag

if ierr == 0
    data.ok = 0;
end

end
