function [ t_mean, t_var ] = calcT( T_sensor1, T_sensor2, R, b_scale)
%GETTRANS Summary of this function goes here
%   Detailed explanation goes here
%   A RANSAC translation selection
%   b_scale = true: has scale problem

N_iter = 1;
N_batch = 1;

%remove transforms with a zero translation
in = [T_sensor1 T_sensor2];
in = in(:,:,all([in(1:3,4,:);in(1:3,8,:)],1));
r = floor(size(in, 3) / N_batch);

t_var = zeros(N_iter,3);
errMin = inf;

% odom_x = []; odom_y = []; odom_z = [];
% for i = 1:size(in, 3)
%     p = in(1:3, 4, i);
% %     p = rotm2tform(R)^(-1)*in(1:4, 1:4, i)*rotm2tform(R);
% %     p = p(1:3, 4);
%     odom_x = [odom_x; p(1)];
%     odom_y = [odom_y; p(2)];
%     odom_z = [odom_z; p(3)];
% end
% figure, plot3(odom_x, odom_y, odom_z, '-r'); title('aligned trajectory, should be same'); hold on;
% % figure, plot(odom_x, odom_y, '-r'); title('aligned trajectory, should be same'); hold on;
% xlabel('x'); ylabel('y'); zlabel('z');
% 
% odom_x = []; odom_y = []; odom_z = [];
% for i = 1:size(in, 3)
%     p = in(1:3, 8, i);
% %     p = rotm2tform(R)*in(1:4, 5:8, i)*rotm2tform(R)^(-1);
% %     p = p(1:3, 4);
%     odom_x = [odom_x; p(1)];
%     odom_y = [odom_y; p(2)];
%     odom_z = [odom_z; p(3)];
% end
% figure, plot3(odom_x, odom_y, odom_z, '-b'); hold off;
% % plot(odom_x, odom_y, '-b'); hold off;

% e = zeros(size(in, 3), 1);
% t = [0 0 0]';
% for i = 1:size(in, 3)
%     e(i) = norm(R*in(1:3, 4, i) + t - in(1:3, 5:7, i)*t + in(1:3, 8, i), 2);
% end
% disp('preprocessing');
% disp(norm(e, 2));

for i = 1:N_iter
%     data = datasample(in,r,3);    
    data = in;
    
%     S = zeros(3*r,r+3);
%     X = zeros(3*r,1);
%     for frame = 1:r
%         S(3*(frame-1)+1:3*(frame-1)+3,1:3) = (data(1:3,1:3,frame)-eye(3));
%         S(3*(frame-1)+1:3*(frame-1)+3,frame+3) = data(1:3,4,frame);    
%         X(3*(frame-1)+1:3*(frame-1)+3) = R*data(1:3,8,frame);
%     end
% 
%     temp = S\X;
%     t_var(i,:) = temp(1:3);

%     % equ.(7) in motion-based paper
    A = zeros(3*r, 3);
    b = zeros(3*r, 1);
    for frame = 1:r
        A(3*(frame-1)+1:3*frame, 1:3) = (data(1:3, 5:7, frame) - eye(3));
        b(3*(frame-1)+1:3*frame) = R*data(1:3, 4, frame) - data(1:3, 8, frame);
    end
    tmp = (A'*A)^(-1) * A' * b;
    t_var(i, :) = tmp;
    
    % find scales and error
    err = zeros(size(T_sensor2,3),1);
    for j = 1:size(T_sensor2,3)
        if b_scale
            s = T_sensor2(1:3,4,j) \ (R*T_sensor1(1:3,4,j) + t_var(i,:)' - ...
                T_sensor2(1:3,1:3,j)*t_var(i,:)');
        else
            s = 1;
        end
        err(j) = norm((R * T_sensor1(1:3,4,j) + t_var(i,:)' - ...
            T_sensor2(1:3, 1:3, j)*t_var(i, :)' - s*T_sensor2(1:3,4,j))).^2;
    end
    
    % select the most accurate translation
    err = median(err);
    if(err < errMin)
        errMin = err;
        t_mean = t_var(i,:);
    end       
end

t_var = (1.4826*mad(t_var,1)).^2;

end

