% =========================================================================
% SUB-ROUTINE LOCALE: FUNZIONE DI COSTO PER OTTIMIZZAZIONE BAYESIANA
% =========================================================================
function J = localCostFunctionInterpolation(params, ptCloudData, res)
xyz = double(ptCloudData.Location);
x = xyz(:,1); y = xyz(:,2); z = xyz(:,3);

N = length(x);
if N > 1000
    idxVal = randperm(N, 1000);
else
    idxVal = 1:N;
end

xTrain = x; yTrain = y; zTrain = z;
xTrain(idxVal) = []; yTrain(idxVal) = []; zTrain(idxVal) = [];

xVal = x(idxVal); yVal = y(idxVal); zVal = z(idxVal);
p = params.idwPower;

zPred = zeros(size(zVal));
for i = 1:length(zVal)
    d = sqrt((xTrain - xVal(i)).^2 + (yTrain - yVal(i)).^2);
    d = max(d, 1e-5);

    % CORRETTO: Uso dell'operatore element-wise (.^) anziché (^)
    w = 1 ./ (d .^ p);

    zPred(i) = sum(w .* zTrain) / sum(w);
end

rmse = sqrt(mean((zVal - zPred).^2, 'omitnan'));
J = rmse;
end