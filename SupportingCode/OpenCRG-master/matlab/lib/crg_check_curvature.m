function [data, ierr, idxArr] = crg_check_curvature(data, ierr)
% CRG_CHECK_CURVATURE Check OpenCRG curvature data.
%   [DATA, IERR, IDXARR] = CRG_CHECK_CURVATURE(DATA) checks reference line curvature in `data`
%   globally and locally. The global curvature check fails if two or more
%   lateral cuts intersect inside the road limits. In this case, the local
%   curvature check still succeeds, if such an intersection falls into a region
%   of NaN values.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%   IERR    number of previous errors
%
%   Outputs:
%   DATA    is a checked, purified, and eventually completed version of
%           the function input argument DATA
%   IERR    summed up number of errors
%   IDXARR  array containing start and end index, where local error occurred
%           if no local error occurred an empty error is returned
%
%   Examples:
%   data = crg_check_curvature(data)
%       Checks CRG reference line curvature.
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


%% some local variables

crgeps = data.opts.ceps;
icerr = 0;
idxArr = [];

%% check for rc field (reference line curvature)

if ~isfield(data, 'rc')
    error('CRG:checkError', 'DATA.rc missing')
end

%% global curvature check
wcvg = 1;
if isfield(data.opts, 'wcvg')
    wcvg = data.opts.wcvg;
end

if wcvg ~= 0
    cmin = min(data.rc);
    cmax = max(data.rc);
    if abs(cmax) > crgeps
        if 1/cmax <= data.head.vmax && 1/cmax >= data.head.vmin
            warning('CRG:checkWarning', 'global curvature check failed - center of max. reference line curvature=%d inside road limits', cmax)
            icerr = icerr + 1;
        end
    end
    if abs(cmin) > crgeps
        if 1/cmin <= data.head.vmax && 1/cmin >= data.head.vmin
            warning('CRG:checkWarning', 'global curvature check failed - center of min. reference line curvature=%d inside road limits', cmin)
            icerr = icerr + 1;
        end
    end
end

ierr = ierr + icerr;

%% local curvature check
wcvl = 0;
if isfield(data.opts, 'wcvl')
    wcvl = data.opts.wcvl;
end

if wcvl > 0 && icerr > 0
    % set temp ok for evaluation
    was_not_ok = 1;
    if isfield(data, 'ok')
        was_not_ok = data.ok;
    end
    data.ok = 0;

    % reference line u-values
    uges = data.head.ubeg:data.head.uinc:data.head.uend;
    uinc = data.head.uinc;

    % reference line radius of curvature
    % no check for rc equals zero before division because IEEE 754 defines 1.0/0.0 = Inf
    vec_r = 1./[data.rc(1),data.rc,data.rc(end)];

    % find radius of curvature in road limits
    r_in_limits = vec_r >= data.head.vmin & vec_r <= data.head.vmax;
    u_in_limits = uges(r_in_limits)';
    v_in_limits = vec_r(r_in_limits)';
    clear vec_r r_in_limits

    % check if any z are non nan where radius of curvature is in limits
    z_in_limits = crg_eval_uv2z(data,[u_in_limits, v_in_limits]);
    z_is_valid  = ~isnan(z_in_limits);

    % if all z are nan then check succeeded
    if ~any(z_is_valid)
        warning('local curvature check succeeded - critical curvature areas in NaN regions')
        ierr = ierr - icerr;
    else
        warning('local curvature check failed - critical curvature areas in z-value regions')
        ierr = ierr + 1;

        % find iu where check fails and store first and last
        u_check_fail = u_in_limits(z_is_valid);
        idxArr = 1 + (u_check_fail([1, end])' - data.head.ubeg)./uinc;
    end

    if was_not_ok || ierr > 0
        % remove ok
        data = rmfield(data, 'ok');
    end
end

end