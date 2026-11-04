function [] = crg_init()
% CRG_INIT Initialize CRG environment.
%   CRG_INIT initializes the CRG MATLAB environment by adding the
%   application directory and required subdirectories to the search path
%
%   Examples:
%   CRG_INIT initializes the CRG environment
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

%% get the application directory

[appdir] = fileparts(mfilename('fullpath'));

%% add application directory and required subdirectories to path

addpath(appdir);
addpath(fullfile(appdir, 'lib'));
addpath(fullfile(appdir, 'test'));
addpath(fullfile(appdir, 'demo'));

end
