function AVTSurf_to_CRG( varargin )

varargin = varargin{1,1};

Zmat   = varargin.ZMat;

uinc   = varargin.uinc;
vinc   = varargin.uinc;

PSname = varargin.PSname;
units  = varargin.units;

% =========================================================
% DIMENSIONI CRG
% =========================================================

[nu, nv] = size(Zmat);

% Estensione longitudinale:
% nu campioni distanziati di uinc -> (nu-1)*uinc
ubeg = 0;
uend = (nu - 1) * uinc;

% Coordinate trasversali centrate su v = 0
v = ((0:nv-1) - (nv-1)/2) * vinc;

vmin = v(1);
vmax = v(end);


% =========================================================
% BUILD THE CRG STRUCTURE
% =========================================================

crgrefl.head.poff = 1;

crgrefl.head.ubeg = single(ubeg);
crgrefl.head.uend = single(uend);
crgrefl.head.uinc = single(uinc);

crgrefl.head.vmin = single(vmin);
crgrefl.head.vmax = single(vmax);
crgrefl.head.vinc = single(vinc);

% Reference line rettilinea
crgrefl.head.xbeg = single(0);
crgrefl.head.xend = single(uend);

crgrefl.head.ybeg = single(0);
crgrefl.head.yend = single(0);

% Matrice delle quote
crgrefl.z = single(Zmat);

% IMPORTANTISSIMO:
% DATA.u deve avere 1 oppure 2 elementi, NON nu elementi
crgrefl.u = single(uend);

% DATA.v invece può contenere tutte le coordinate trasversali
crgrefl.v = single(v);

crgrefl.uinc = single(uinc);
crgrefl.ubeg = single(ubeg);
crgrefl.uend = single(uend);

% crgrefl.v         = single(1);
crgrefl           = crg_check( crgrefl );

% crgrefl.head      = rmfield( crgrefl.head, 'vinc' );
crgrefl.ct        = { PSname };
    
crgrefl.opts.bdmu = 0;
crgrefl.opts.bdmv = 0;

finalData         = crgrefl;

finalData = crg_check(finalData);

%  ADD METADATA TO THE CRG FILE  ==========================================

CRG_name = [ PSname, '.crg' ];
finalData.ct{1} = 'CRG defined by Z matrix';

finalData = crg_check( finalData );

%  modify name in CRG struct
finalData.ct{1} = CRG_name;
finalData.ct{2} = pwd;
finalData.filenm = CRG_name;

finalData.units = units;
finalData.opts.bdmu = uint8( 2 );
finalData.opts.bdmv = uint8( 2 );

%  Create folder to save output GIS data
if ~exist( [ pwd, '\AVT_Out\' ], 'dir' )
    mkdir( [ pwd, '\AVT_Out\' ] );
end

%  Create CRG file
%
%  error in crg_write.m, line 78.  Change to:
% for i = 1:numel(f)

crg_write( finalData, CRG_name );
destination = [ 'AVT_Out\', CRG_name ];
movefile( CRG_name, destination );

%  Create CRG report
% crg_show_all( finalData )
% movefile( strrep(CRG_name,'crg','pdf'), ...
%     strrep(destination,'crg','pdf') );
% 
% winopen( strrep(destination,'crg','pdf') );