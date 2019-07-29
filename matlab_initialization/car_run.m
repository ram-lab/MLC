clc; close all;
addpath('./yaml');
addpath('./kabsch');

%% parameter setting
str_gt_mat = 'data/yellow_20180104';
str_odom_obtain = 'data/simulation_odom_2';
str_odom_bag = 'read_from_bag'; % 'calculate_from_pcd'
b_odom_obtain = false;
bag_file = '/Monster/dataset/lidar_calibration/simulation_20181128/loop_1.bag';
b_abs_rel_odom_check = false;
b_plot_odom = false;

disp('Load ground truth ...');
load(str_gt_mat);
% Tform_gt_lidar_1_lidar_2 = ...
%     [1.0, 0.0, 0.0, 2.2; ...
%      0.0, 1.0, 0.0, 0; ...
%      0.0, 0.0, 1.0, -0.8; ...
%      0.0, 0.0, 0.0, 1];
% Tform_gt_lidar_1_lidar_3 = ...
%     [-1.0, 0.0, 0.0, -2.0; ...
%      0.0, -1.0, 0.0, 0.0; ...
%      0.0, 0.0, 1.0, -1.0; ...
%      0.0, 0.0, 0.0, 1.0];

%% obtain odom
if b_odom_obtain
    disp('Load odom ...');
    load(str_odom_obtain);
else
    if strcmp(str_odom_bag, 'read_from_bag')
        disp('Read odom from bag ...');        
        bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/8_shape/odom_top.bag';
        lidar_1_data = odom_obtain_from_bag(bag_file, 'Transformation of top lidar', 1, '/odom_from_start', b_plot_odom, b_noise);
        bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/8_shape/odom_front.bag';
        lidar_2_data = odom_obtain_from_bag(bag_file, 'Transformation of front lidar', 2, '/odom_from_start', b_plot_odom, b_noise);
        bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/8_shape/odom_tail.bag';
        lidar_3_data = odom_obtain_from_bag(bag_file, 'Transformation of tail lidar', 3, '/odom_from_start', b_plot_odom, b_noise);
    elseif strcmp(str_odom_bag, 'calculate_from_pcd')
        fprintf('Calculate odom from pcd ...');
        data_path = '/Monster/dataset/lidar_calibration/yellow_lidar_calibration_20181124/pcd/';
        lidar_1_data = gen_lidar_tf([data_path, 'top/'], true, 'TOP');
        lidar_2_data = gen_lidar_tf([data_path, 'front/'], true, 'FRONT');
        lidar_3_data = gen_lidar_tf([data_path, 'tail/'], true, 'TAIL');
    end
end
figure;
plot_sensor_odom(lidar_1_data, 'Transformation of top lidar', 3, 1);
plot_sensor_odom(lidar_2_data, 'Transformation of front lidar', 3, 2);
plot_sensor_odom(lidar_3_data, 'Transformation of tail lidar', 3, 3);

if b_abs_rel_odom_check
    disp(lidar_1_data.tformAbs_lidar(:,:,end));
    tform = eye(4);
    for frame = 1:length(lidar_1_data.tformRel)
        tform = tform * lidar_1_data.tformRel(:,:,frame);
    end
    disp(tform); disp('checking if equal?');
end

disp('Finish odom acquirement !');

%% obtain R and T
%% lidar_1 -> lidar_2
disp('lidar_1 to lidar_2 ...');
l = min(length(lidar_1_data.tformRel), length(lidar_2_data.tformRel));
lidar_1_Tform = lidar_1_data.tformRel(:,:,1:l);
lidar_2_Tform = lidar_2_data.tformRel(:,:,1:l);

[Rmean, Rvar] = calcR(lidar_1_Tform, lidar_2_Tform);
% disp('Rmean'); disp(Rmean);
% disp('R_gt'); disp(Tform_gt_lidar_1_lidar_2(1:3, 1:3));

[Tmean, Tvar] = calcT(lidar_1_Tform, lidar_2_Tform, Rmean, false);

% [r1,r2,r3] = dcm2angle(Rmean);
% out = [180*[r1,r2,r3]/pi, Tmean];
% fprintf('angle: %f %f %f translation: %f %f %f\n',out(1),out(2),out(3),out(4),out(5),out(6));

Tform_lidar_1_lidar_2 = [[Rmean,Tmean'];0,0,0,1];

disp('Guess: lidar_1 to lidar_2'); disp(Tform_lidar_1_lidar_2);
disp('GT: lidar_1 to lidar_2'); disp(Tform_gt_lidar_1_lidar_2);
disp('Evaulation: error');
[e_x, e_y, e_z, e_roll, e_pitch, e_yaw] = calculate_error(Tform_gt_lidar_1_lidar_2, Tform_lidar_1_lidar_2)

%% lidar_1 -> lidar_3
disp('lidar_1 to lidar_3 ...');
l = min(length(lidar_1_data.tformRel), length(lidar_3_data.tformRel));
lidar_1_Tform = lidar_1_data.tformRel(:,:,1:l); 
lidar_3_Tform = lidar_3_data.tformRel(:,:,1:l);

[Rmean, Rvar] = calcR(lidar_1_Tform, lidar_3_Tform);
% disp('Rmean'); disp(Rmean);
% disp('R_gt'); disp(Tform_gt_lidar_1_lidar_3(1:3, 1:3));

[Tmean, Tvar] = calcT(lidar_1_Tform, lidar_3_Tform, Rmean, false);

% [r1,r2,r3] = dcm2angle(Rmean);
% out = [180*[r1,r2,r3]/pi, Tmean];
% fprintf('angle: %f %f %f translation: %f %f %f\n',out(1),out(2),out(3),out(4),out(5),out(6));

Tform_lidar_1_lidar_3 = [[Rmean,Tmean'];0,0,0,1];

disp('Guess: lidar_1 to lidar_3'); disp(Tform_lidar_1_lidar_3);
disp('GT: lidar_1 to lidar_3'); disp(Tform_gt_lidar_1_lidar_3);
disp('Evaulation: error');
[e_x, e_y, e_z, e_roll, e_pitch, e_yaw] = calculate_error(Tform_gt_lidar_1_lidar_3, Tform_lidar_1_lidar_3)







