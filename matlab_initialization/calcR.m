function [ M, V, err ] = calcR( T_sensor_1, T_sensor_2)
% LMEDSR get least median of squares rotation
% TODO: RANSAC

% TODO
N_iter = 500;
sub = 30;

T1R = zeros(size(T_sensor_2,3),4); % rotation vector Nx4
T1D = zeros(size(T_sensor_2,3),4); % normalize translation Nx4
T2R = zeros(size(T_sensor_1,3),4);
T2D = zeros(size(T_sensor_1,3),4);

for i = 1:size(T_sensor_1,3)
    T1R(i,:) = vrrotmat2vec(T_sensor_2(1:3,1:3,i));
    T2R(i,:) = vrrotmat2vec(T_sensor_1(1:3,1:3,i));

    T1R(i,1:3) = T1R(i,1:3)*T1R(i,4);
    T2R(i,1:3) = T2R(i,1:3)*T2R(i,4);
    
    T1D(i,1:3) = T_sensor_2(1:3,4,i)./norm(T_sensor_2(1:3,4,i));  
    T2D(i,1:3) = T_sensor_1(1:3,4,i)./norm(T_sensor_1(1:3,4,i));
    
    T1D(i,1:3) = T1D(i,1:3);
    T2D(i,1:3) = T2D(i,1:3);
end

T1D(~isfinite(T1D(:))) = 0;
T2D(~isfinite(T2D(:))) = 0;
T1R(~isfinite(T1R(:))) = 0;
T2R(~isfinite(T2R(:))) = 0;

%% Kabsch input are translation vectors
dV = zeros(3,3,N_iter);
errMinD = inf;
for i = 1:N_iter
    data = datasample([T2D(:,1:3) T1D(:,1:3)],sub,1);
    dV(:,:,i) = Kabsch(data(:,1:3)',data(:,4:6)'); % ref, data
    rotValidate(dV(:,:,i));
    err = median(sum((dV(:,:,i)*T2D(:,1:3)'-T1D(:,1:3)').^2,1)); % RMSD
    
    if(err < errMinD)
        errMinD = err;
        dM = dV(:,:,i); % optimal rotation result of translation value
    end
    
end
dV = (1.4826*mad(dV,1,3)).^2; % covariance

%% Kabsch input are rotation vectors
rV = zeros(3,3,N_iter);
errMinR = inf;
for i = 1:N_iter
    data = datasample([T2R(:,1:3) T1R(:,1:3)],sub,1);
    rV(:,:,i) = Kabsch(data(:,1:3)',data(:,4:6)');
    rotValidate(rV(:,:,i));
    err = median(sum((rV(:,:,i)*T2R(:,1:3)'-T1R(:,1:3)').^2,1));
    
    if(err < errMinR)
        errMinR = err;
        rM = rV(:,:,i); % optimal rotation result of rotation value
    end
    
end
rV = (1.4826*mad(rV,1,3)).^2; % covariance

%%
err = errMinR + errMinD;

%combine results into one measure, weighting
dW = (1./dV)./((1./rV)+(1./dV));
rW = (1./rV)./((1./rV)+(1./dV));

M = dM.*dW + rM.*rW;
V = dV.*dW + rV.*rW;

% reorthogonalize
[a,b,c] = svd(M);
temp = a*c';
if(det(temp) < 0)
    [~,idx] = min(sum(M.^2));
    M = temp;
    M(:,idx) = -M(:,idx);
end

M = rM;
V = rV;





