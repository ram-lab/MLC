clc; close all;
%% user set variables
bag_select = rosbag('/Monster/dataset/lidar_calibration/golf_cart_lidar_calibration_20181119/loop_odom_2018-11-22-11-43-58.bag');
R_sensor1_sensor2 = eul2rotm([0 0 0], 'ZYZ');
t_sensor1_sensor2 = [0 0 2];
T_sensor1_sensor2 = [[R_sensor1_sensor2, t_sensor1_sensor2'];0,0,0,1];

%% add external function directories
% addpath('./libicp/matlab');
% addpath('./psopt');
addpath('./kabsch');

%% process odom
topic_select = select(bag_select, 'Time', ...
    [1542858246.14 1542858400.88], 'Topic', '/odom_from_start');
odom_msg = readMessages(topic_select, 'DataFormat', 'struct');

sensor1_Tform = zeros(4, 4, length(1:20:length(odom_msg)));
odom_x = []; odom_y = []; odom_z = [];
for i = 1:20:length(odom_msg)
    tform = eye(4,4);
    tform(1:3, 4) = [odom_msg{i}.Pose.Pose.Position.X, ...
        odom_msg{i}.Pose.Pose.Position.Y, odom_msg{i}.Pose.Pose.Position.Z];
    tform(1:3, 1:3) = quat2rotm([odom_msg{i}.Pose.Pose.Orientation.W, odom_msg{i}.Pose.Pose.Orientation.X, ...
        odom_msg{i}.Pose.Pose.Orientation.Y, odom_msg{i}.Pose.Pose.Orientation.Z]);
    sensor1_Tform(:, :, floor(i/20)+1) = tform;
    odom_x = [odom_x;tform(1,4)]; odom_y = [odom_y;tform(2,4)]; odom_z = [odom_z;tform(3,4)];
end
% figure, plot3(odom_x, odom_y, odom_z, '-r'); title('split trajectory, should be different');
figure, plot(odom_x, odom_y, '-r'); title('split trajectory, should be different'); hold on;
xlabel('x'); ylabel('y'); zlabel('z');

sensor2_Tform = zeros(4, 4, length(1:20:length(odom_msg)));
odom_x = []; odom_y = []; odom_z = [];
for i = 1:20:length(odom_msg)
    tform = eye(4,4);
    tform(1:3, 4) = [odom_msg{i}.Pose.Pose.Position.X, ...
        odom_msg{i}.Pose.Pose.Position.Y, odom_msg{i}.Pose.Pose.Position.Z];
    tform(1:3, 1:3) = quat2rotm([odom_msg{i}.Pose.Pose.Orientation.W, odom_msg{i}.Pose.Pose.Orientation.X, ...
        odom_msg{i}.Pose.Pose.Orientation.Y, odom_msg{i}.Pose.Pose.Orientation.Z]);
    tform = T_sensor1_sensor2^(-1) * tform * T_sensor1_sensor2; % left multiplication
    sensor2_Tform(:, :, floor(i/20)+1) = tform;
    odom_x = [odom_x;tform(1,4)]; odom_y = [odom_y;tform(2,4)]; odom_z = [odom_z;tform(3,4)];
end
% figure, plot3(odom_x, odom_y, odom_z, 'b'); title('split trajectory, should be different');
figure, plot(odom_x, odom_y, '-b'); title('split trajectory, should be different'); 
xlabel('x'); ylabel('y'); zlabel('z');

%% transform odom to relative transformation
sensor1_Tform_rel = zeros(4, 4, size(sensor1_Tform, 3)-1);
for i = 1:size(sensor1_Tform, 3)-1
    sensor1_Tform_rel(:, :, i) = sensor1_Tform(:, :, i+1) * sensor1_Tform(:, :, i)^(-1);
end

sensor2_Tform_rel = zeros(4, 4, size(sensor2_Tform, 3)-1);
for i = 1:size(sensor2_Tform, 3)-1
    sensor2_Tform_rel(:, :, i) = sensor2_Tform(:, :, i+1) * sensor2_Tform(:, :, i)^(-1);
end


%% find transforms
[Rmean, Rvar] = calcR(sensor2_Tform_rel, sensor1_Tform_rel);
disp('Rmean'); disp(Rmean);
disp('R_gt'); disp(R_sensor1_sensor2);

[Tmean, Tvar] = calcT(sensor1_Tform_rel, sensor2_Tform_rel, Rmean, false);

[r1,r2,r3] = dcm2angle(Rmean);
out = [180*[r1,r2,r3]/pi, Tmean];
fprintf('angle: %f %f %f translation: %f %f %f\n',out(1),out(2),out(3),out(4),out(5),out(6));

%% new tform
initial_guess = [[Rmean,Tmean'];0,0,0,1];

disp('Groundtruth');
disp(T_sensor1_sensor2);
disp('Guess');
disp(initial_guess);








