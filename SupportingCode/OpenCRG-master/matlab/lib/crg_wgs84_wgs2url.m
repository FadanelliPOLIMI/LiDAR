function [url] = crg_wgs84_wgs2url(wgs, opts)
%CRG_WGS84_WGS2URL Generate URL to show WGS-84 information on Google Maps
%   [URL] = CRG_WGS84_WGS2URL(WGS, OPTS) generates a URL
%   for showing WGS-84 coordinates using Google Maps.
%
%   Inputs:
%   WGS     (np, 2) arrays of latitude/longitude (north/east) WGS-84
%           coordinate pairs (in degrees)
%   OPTS    struct for method options (optional, no default)
%   .label  sets label comment text (default: 'OpenCRG road mark')
%
%   Outputs:
%   URL     URL string (for np=1) or struct of strings (for np>1)
%
%   Examples:
%   wgs = [51.477811,-0.001475]  % Greenwich Prime Meridian
%                                % (Airy's Transit Circle)
%   url = crg_wgs84_wgs2url(wgs) % generate url string
%   web(url, '-browser')         % show URL in default browser
%   Generate URL to show WGS-84 info using Google Maps
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

%% handle optional arguments

if nargin < 2
    opts = struct;
end

if isfield(opts, 'label')
    label = opts.label;
else
    label = 'OpenCRG road mark';
end
label = strrep(label, ' ', '+'); % replace blanks by '+' for url string

%% generate URL string

np = size(wgs, 1);

if np == 1
    url = sprintf('http://maps.google.com/maps?q=%.6f,%.6f(%s)', wgs(1,1), wgs(1,2), label);
else
    for i = 1:np
        url{i} = sprintf('http://maps.google.com/maps?q=%.6f,%.6f(%s)', wgs(i,1), wgs(i,2), label); %#ok<AGROW>
    end
end

end
