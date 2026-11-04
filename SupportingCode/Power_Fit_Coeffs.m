function [ pow, int ] = Power_Fit_Coeffs( w, psd, CutOff )

% Discard first and last points
% if numel( w ) > 3
%     w   = w(1:(end-1));
%     psd = psd(1:(end-1));
% end

%  drop data beyond CutOff
if CutOff < 9999
    ind = w > (1/CutOff);
    w = w( ind );
    psd = psd( ind );
end

% Power fit
xv = log10( w );  
yv = log10( psd );
[ coeffs1, ~ ] = polyfit( xv, yv, 1 ); % norm
pow = coeffs1(1,1);
int = 10^( coeffs1(1,2) );