SSmat = load( 'NewMat.mat' );
SSmat = SSmat.output;
sigma = SSmat(:,1);
RMSfc = SSmat(:,2);
pow = SSmat(:,3);
CutOff = SSmat(:,4);
H = SSmat(:,5);

f1 = figure( 'Theme', 'light', 'MenuBar', 'none', 'ToolBar', 'none' );
        f1.PaperOrientation = 'portrait';
        f1.WindowState   = 'normal';
        f1.Units      = 'inches';
        f1.Position   = [ 0 0 8.5 11 ];
        f1.PaperUnits = 'inches';
        f1.PaperSize  = [ 8.5 11 ];
        f1.PaperPosition = [ 0 0 8.5 11 ];
        movegui('center')
        pause( 0.5 );

t = tiledlayout(4,2)

nexttile(t,1,[2 2])
scatter3(RMSfc,pow,CutOff,[],H)
title( {'Synthetic Suface Design Space';'Color is Hurst exponent'})
xlabel('CutOff Limited RMS (in)');
ylabel( 'WNS exponent' );
zlabel('CutOff (ft)');
box on
colorbar

nexttile(t,5,[1 1]);
scatter(sigma,RMSfc,[],H);
title( {'CutOff Limited RMS vs. Input RMS'})
ylabel('CutOff Limited RMS (in)');
xlabel('Input RMS (in)');
box on
grid on

nexttile(t,6,[1 1]);
scatter(H,pow,[],H);
title( {'WNS Exponent vs. H'})
ylabel('WNS exponent');
xlabel('H');
box on
grid on


nexttile(t,7,[1 1]);
histogram2(CutOff,RMSfc);
title( {'CutOff Limited RMS vs. CutOff'})
ylabel({'CutOff Limited';'RMS (in)'});
xlabel('CutOff (ft)');
zlabel('Count');


nexttile(t,8,[1 1]);
histogram2(H,RMSfc);
title({'CutOff Limited RMS vs. H'})
ylabel({'CutOff Limitted'; 'RMS (in)'});
xlabel('H');
zlabel('Count');


exportgraphics(f1, 'MatrixVisual.pdf' ,'Resolution',600,'Append',true)
















