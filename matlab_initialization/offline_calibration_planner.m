%% Reference
% 1. hand-eye calibration:
%   CamOdoCal: Automatic intrinsic and extrinsic calibration of a rig with multiple generic cameras and odometry
%   @see https://www.inf.ethz.ch/personal/pomarc/pubs/HengIROS13.pdf
%   Daniilidis, Konstantinos. "Hand-eye calibration using dual quaternions." The International Journal of Robotics Research 18.3 (1999): 286-298.
%   @see <a href="http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.136.5873&rank=1">Daniilidis 1999</a>

%%
clc;
% close all;
addpath('./yaml');
addpath('./kabsch');
addpath('./camodocal');

%% parameter setting
str_gt_mat = 'data/yellow_20180104';
% str_gt_mat = 'data/simulation_carla_gt';
% str_odom_obtain = 'data/simulation_odom_2';
str_odom_bag = 'read_from_bag'; % 'calculate_from_pcd'
str_initializer = 'quaternion';
% str_initializer = 'matrix';
b_odom_obtain = false;
b_abs_rel_odom_check = false;
b_plot_odom = false;
b_plot_sensor_odom = true;
b_noise = false;

disp('Load ground truth ...');
load(str_gt_mat);

rng(2);

top_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_top.bag';
front_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_front.bag';
tail_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_tail.bag';

% top_file = '/Monster/dataset/lidar_calibration/simulation_20181128/loop_1_transformed.bag';
% front_file = '/Monster/dataset/lidar_calibration/simulation_20181128/loop_1_transformed.bag';
% tail_file = '/Monster/dataset/lidar_calibration/simulation_20181128/loop_1_transformed.bag';

%% obtain odom
fprintf('Read odom from bag ...\n');
lidar_1_data = odom_obtain_from_bag(top_file, 'Transformation of top lidar', 1, '/odom_from_start', b_plot_odom, b_noise);
lidar_2_data = odom_obtain_from_bag(front_file, 'Transformation of front lidar', 2, '/odom_from_start', b_plot_odom, b_noise);
lidar_3_data = odom_obtain_from_bag(tail_file, 'Transformation of tail lidar', 3, '/odom_from_start', b_plot_odom, b_noise);
if b_plot_sensor_odom
    figure(1);
    plot_sensor_odom(lidar_1_data, 'Transformation of top lidar', 3, 1);
    plot_sensor_odom(lidar_2_data, 'Transformation of front lidar', 3, 2);
    plot_sensor_odom(lidar_3_data, 'Transformation of tail lidar', 3, 3);
end
fprintf('Finish odom acquirement !\n');

% TODO: transform coordinate
% for i = 1:length(lidar_1_data.tformAbs_world)
%     tmp = lidar_1_data.tformAbs_world(1, 4, i);
%     lidar_1_data.tformAbs_world(1, 4, i) = lidar_1_data.tformAbs_world(3, 4, i);
%     lidar_1_data.tformAbs_world(3, 4, i) = lidar_1_data.tformAbs_world(2, 4, i);
%     lidar_1_data.tformAbs_world(2, 4, i) = tmp;
% end
%
% for i = 1:length(lidar_2_data.tformAbs_world)
%     tmp = lidar_2_data.tformAbs_world(1, 4, i);
%     lidar_2_data.tformAbs_world(1, 4, i) = lidar_2_data.tformAbs_world(3, 4, i);
%     lidar_2_data.tformAbs_world(3, 4, i) = lidar_2_data.tformAbs_world(2, 4, i);
%     lidar_2_data.tformAbs_world(2, 4, i) = tmp;
% end
% for i = 1:length(lidar_3_data.tformAbs_world)
%     tmp = lidar_3_data.tformAbs_world(1, 4, i);
%     lidar_3_data.tformAbs_world(1, 4, i) = lidar_3_data.tformAbs_world(3, 4, i);
%     lidar_3_data.tformAbs_world(3, 4, i) = lidar_3_data.tformAbs_world(2, 4, i);
%     lidar_3_data.tformAbs_world(2, 4, i) = tmp;
% end

%% step 2: Initialization
fprintf('lidar_1 to lidar_2 ...\n');
[X_12, x_kabsch_12, x_proposed_12] = initialization_planar(lidar_1_data, lidar_2_data);

fprintf('lidar_1 to lidar_3 ...\n');
[X_13, x_kabsch_13, x_proposed_13] = initialization_planar(lidar_1_data, lidar_3_data);

save("data/initialization/simulate_1_noise_2.mat", 'X_12', 'x_kabsch_12', 'x_proposed_12', 'X_13', 'x_kabsch_13', 'x_proposed_13');

%% step 3: Ground alignment
read_from_mat = true;
% str_ini_X_mat = 'data/eclipse_ini_X.mat';
str_ground_centroid_mat = 'data/eclipse_ground_centroid.mat';
% load(str_ini_X_mat);

if read_from_mat
    load(str_ground_centroid_mat);
    lidar_1_data.ground_centroid = ground_centroid_1;
    lidar_2_data.ground_centroid = ground_centroid_2;
    lidar_3_data.ground_centroid = ground_centroid_3;
else
    bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_top.bag';
    lidar_1_data.ground_centroid = ground_obtain_from_bag(bag_file, '/ground_cloud');
    bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_front.bag';
    lidar_2_data.ground_centroid = ground_obtain_from_bag(bag_file, '/ground_cloud');
    bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_tail.bag';
    lidar_3_data.ground_centroid = ground_obtain_from_bag(bag_file, '/ground_cloud');
end
R = ini_X(1:3,1:3);
v_t = [];
gc_1 = datasample(lidar_1_data.ground_centroid, floor(length(lidar_1_data.ground_centroid)/10));
gc_3 = datasample(lidar_3_data.ground_centroid, floor(length(lidar_3_data.ground_centroid)/10));
for i = 1:length(gc_1)
    p1 = gc_1{i}';
    p3 = gc_3{i}';
    t = p1 - R*p3;
    v_t = [v_t, p1 - R*p3];
end
v_mean_t = mean(v_t, 2);
v_std_t = std(v_t, 0, 2);
disp('t_z mean:'); disp(v_mean_t(3));
disp('t_z std:'); disp(v_std_t(3));
ini_X(3, 4) = v_mean_t(3);

disp('# INFO: Motion-based after ground alignment refinement: ');
rotm2eul(ini_X(1:3,1:3))/pi*180
disp('Guess: lidar_1 to lidar_3'); disp(ini_X);
disp('GT: lidar_1 to lidar_3'); disp(Tform_gt_lidar_1_lidar_3);

%% step 4: Miscalibration detection
% ini_X = Tform_gt_lidar_1_lidar_3;
for i = 1:length(lidar_1_data.tformRel_val)
    lidar_1_Tform = lidar_1_data.tformRel_val(:, :, i);
    lidar_3_Tform = lidar_3_data.tformRel_val(:, :, i);

    % error before calibration
    [r_error, t_error] = miscalibration_error(lidar_1_Tform, lidar_3_Tform, X, 'rad');
    rotation_error_wo_calibration(i) = r_error;
    translation_error_wo_calibration(i) = t_error;

    % error after calibration
    [r_error, t_error] = miscalibration_error(lidar_1_Tform, lidar_3_Tform, ini_X,'rad');
    rotation_error_calibration(i) = r_error;
    translation_error_calibration(i) = t_error;
    if (r_error > r_epsilon)
        count_rotation_error = count_rotation_error + 1;
    end

end

%% step 5: Visualizing the calibration results
figure;
subplot(2,3,1); title('Screw motion (rotation)'); hold on; plot(1:1:l, theta_error, 'r-'); hold off;
subplot(2,3,4); title('Screw motion (translation)'); hold on; plot(1:1:l, dis_error, 'b-'); hold off;
subplot(2,3,2); title('Before calibration: rotation error [rad]'); hold on; plot(1:1:l, rotation_error_wo_calibration, 'r-'); hold off;
subplot(2,3,5); title('Before calibration: translation error [m]'); hold on; plot(1:1:l, translation_error_wo_calibration, 'b-'); hold off;
subplot(2,3,3); title('After calibration: rotation error [rad]'); hold on; plot(1:1:l, rotation_error_calibration, 'r-'); hold off;
subplot(2,3,6); title('After calibration: translation error [m]'); hold on; plot(1:1:l, translation_error_calibration, 'b-'); hold off;

%% step 6: Visualizing the calibration results
figure;
subplot(1,2,1); title(''); hold on; plot(1:1:l, theta_error, '-', 'Color', [0, 0, 153]/255); hold off;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 25);
set(gca,'position',[0.1,0.2,0.35,0.45] );
xlabel('Timestamp');
ylabel('Error [rad]')
grid on;


subplot(1,2,2); title(''); hold on; plot(1:1:l, dis_error, '-',  'Color', [190,10,50]/255); hold off;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 25);
set(gca,'position',[0.55,0.2,0.35,0.45] );
xlabel('Timestamp');
ylabel('Error [m]')
grid on;

close all;
% step 5: appearance-based refinement

% an online pipeline
% follow the work: Monocular visual-inertial fusion with online initialization and camera-IMU calibration
% estimate_Ryx
%   if converaged? -> stop calibrating rotation, start calibrating Rz, tx, ty
% nonlinear optimization refinementa
%   if converaged? -> stop calibrating
% appearance-based refinement
