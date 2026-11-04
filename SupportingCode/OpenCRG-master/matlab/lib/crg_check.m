function [data] = crg_check(data)
% CRG_CHECK Run all OpenCRG data checks.
%   [DATA] = CRG_CHECK(DATA) runs all available checks for the given OpenCRG data.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    is a checked, cleaned-up, and potentially completed version of
%           the input DATA.
%
%   Examples:
%   data = crg_check(data)
%   Runs all checks on data.
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

%% initialize error counter

ierr = 0;

%% check options for consistency

data = crg_check_opts(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% check modifiers for consistency

data = crg_check_mods(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% check head consistency

data = crg_check_head(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% check data consistency

data = crg_check_data(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% check map projection data consistency

data = crg_check_mpro(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% check mapping consistency

data = crg_check_wgs84(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% check core data type

data = crg_check_single(data);
if ~isfield(data, 'ok')
    ierr = ierr + 1;
end

%% set ok-flag

if ierr == 0
    data.ok = 0;
else
    warning('CRG:checkWarning', 'check of DATA was not completely successful')
    if isfield(data, 'ok')
        data = rmfield(data, 'ok');
    end
end

end
