clc; close all;
%% add external function directories
addpath('./yaml');
addpath('./kabsch');

%% process bag
figure;
bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/8_shape/odom_top.bag';
comment = 'Trajectory of top lidar';
[sensor1_Tform] = bag_process_odom(bag_file, comment, 1, '/odom_from_start');

bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/8_shape/odom_front.bag';
comment = 'Trajectory of front lidar';
[sensor2_Tform] = bag_process_odom(bag_file, comment, 2, '/odom_from_start');

bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/8_shape/odom_tail.bag';
comment = 'Trajectory of tail lidar';
[sensor3_Tform] = bag_process_odom(bag_file, comment, 3, '/odom_from_start');


%% transform odom to relative transformation
sensor1_Tform_rel = zeros(4, 4, size(sensor1_Tform, 3)-1);
for i = 1:size(sensor1_Tform, 3)-1
    sensor1_Tform_rel(:, :, i) = sensor1_Tform(:, :, i+1) * sensor1_Tform(:, :, i)^(-1);
end

sensor2_Tform_rel = zeros(4, 4, size(sensor2_Tform, 3)-1);
for i = 1:size(sensor2_Tform, 3)-1
    sensor2_Tform_rel(:, :, i) = sensor2_Tform(:, :, i+1) * sensor2_Tform(:, :, i)^(-1);
end

sensor3_Tform_rel = zeros(4, 4, size(sensor3_Tform, 3)-1);
for i = 1:size(sensor2_Tform, 3)-1
    sensor3_Tform_rel(:, :, i) = sensor3_Tform(:, :, i+1) * sensor3_Tform(:, :, i)^(-1);
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








