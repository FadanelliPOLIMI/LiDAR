function [pz, data] = crg_eval_xy2z(data, pxy)
%CRG__EVAL_XY2Z Evaluate z at x/y-position.
%   [PZ, DATA] = CRG_EVAL_XY2Z(DATA, PXY) evaluates the z-values at
%   the given x/y-positions. This function combines the calls of
%   CRG_EVAL_XY2UV and CRG_EVAL_UV2Z.
%
%   inputs:
%       DATA    struct array as defined in CRG_INTRO.
%       PXY     (np, 2) array of points in xy system
%
%   outputs:
%       PZ      (np) vector of z values
%       DATA    struct array as defined in CRG_INTRO, with history added
%
%   Examples:
%   [pz, data] = crg_eval_xy2z(data, pxy)
%       Evaluates the z-values at the given x/y-positions.
%
%   See also CRG_EVAL_XY2UV, CRG_EVAL_UV2Z, CRG_INTRO.

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

%% just do it

[puv, data] = crg_eval_xy2uv(data, pxy);
[pz , data] = crg_eval_uv2z (data, puv);

end
