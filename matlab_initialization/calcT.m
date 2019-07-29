function [ Tmean, Tvar ] = calcT(T_sensor_1, T_sensor_2, R, b_scale)
%GETTRANS Summary of this function goes here
%   Detailed explanation goes here

N_iter = 500;
r = floor(length(T_sensor_1) / 2);
% r = 50;

%remove transforms with a zero translation
in = [T_sensor_1 T_sensor_2];
in = in(:,:,all([in(1:3,4,:);in(1:3,8,:)],1));

Tvar = zeros(N_iter,3);
errMin = inf;
for i = 1:N_iter
    data = datasample(in,r,3);
    
    S = zeros(3*r,4);
    X = zeros(3*r,1);    
    for frame = 1:r
        S(3*(frame-1)+1:3*(frame-1)+3,1:3) = (data(1:3,1:3,frame)-eye(3));
        S(3*(frame-1)+1:3*(frame-1)+3,4) = data(1:3,4,frame);
        X(3*(frame-1)+1:3*(frame-1)+3) = R*data(1:3,8,frame);
    end

    temp = S\X; % S*temp=X, temp:4x1
    Tvar(i,:) = temp(1:3);
    
    %find scales and error
    err = zeros(size(T_sensor_2,3),1);
    for j = 1:size(T_sensor_2,3)
        if(any(T_sensor_2(1:3,4,j)))
            if b_scale
                s = T_sensor_2(1:3,4,j)\(R*T_sensor_1(1:3,4,j) + Tvar(i,:)' - T_sensor_2(1:3,1:3,j)*Tvar(i,:)');
            else
                s = 1;
            end
            err(j) = sum((R*T_sensor_1(1:3,4,j) + Tvar(i,:)' - T_sensor_2(1:3,1:3,j)*Tvar(i,:)' - s*T_sensor_2(1:3,4,j)).^2);
        else
            err(j) = 0;
        end        
    end
    
    err = median(err);
    if(err < errMin)
        errMin = err;
        Tmean = Tvar(i,:);
    end        
end

Tvar = (1.4826*mad(Tvar,1)).^2;

end
