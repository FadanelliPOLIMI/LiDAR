function [] = crg_demo_gen_syntheticStraight()
% CRG_DEMO_GEN_SYNTHETICSTRAIGHTREFLINE CRG demo to generate a synthetic straight OpenCRG file.
%   CRG_DEMO_GEN_SYNTHETICSTRAIGHT() demonstrates how a simple straight OpenCRG file can be
%   generated.
%
%   Example:
%   crg_demo_gen_syntheticStraight    runs this demo to generate "simpleStraight.crg"
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

%% default settings

u =   [  0    900   ];
v =   [ -2.50   2.5 ];
inc = [  0.04   0.02];

filename = 'simpleStraight.crg';
ct = 'CRG generated file';

%% generate synthetical straight OpenCRG file

data = crg_gen_csb2crg0(inc, u, v);

%% add z-values

[nu nv] = size(data.z);

z = 0.01*peaks(nv);
z = repmat(z, ceil(nu/nv), 1);

data.z(1:nu,:) = single(z(1:nu,:));

%% write to file

data.ct{1} = ct;
crg_write(data, filename);

%% display result

crg_show(data);

end
