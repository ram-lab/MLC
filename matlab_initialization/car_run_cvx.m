clc; close all;
addpath('./yaml');
addpath('./kabsch');

%% parameter setting
str_gt_mat = 'data/simulation_transformation_gt';
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
        lidar_1_data = odom_obtain_from_bag(bag_file, 'Transformation of top lidar', 1, '/top_laser_ground_truth', b_plot_odom);
        lidar_2_data = odom_obtain_from_bag(bag_file, 'Transformation of front lidar', 2, '/front_laser_ground_truth', b_plot_odom);
        lidar_3_data = odom_obtain_from_bag(bag_file, 'Transformation of tail lidar', 3, '/tail_laser_ground_truth', b_plot_odom);
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
l = min(min(length(lidar_1_data.tformRel), length(lidar_2_data.tformRel)), length(lidar_3_data.tformRel));
v_t = 1:1:l/40;
r_initialization = 1;

%% lidar_1 -> lidar_2
[m_e_x, m_e_y, m_e_z, m_e_roll, m_e_pitch, m_e_yaw] = deal(zeros(r_initialization, length(v_t)));
v_str_xlabel = cell(1,length(v_t));
disp('lidar_1 to lidar_2 ...');
for t = 1:length(v_t)
    for r = 1:r_initialization
        lidar_1_Tform = lidar_1_data.tformRel(:,:,1:v_t(t)*40);
        lidar_2_Tform = lidar_2_data.tformRel(:,:,1:v_t(t)*40);
        [Rmean, Rvar] = calcR(lidar_1_Tform, lidar_2_Tform);
        [Tmean, Tvar] = calcT(lidar_1_Tform, lidar_2_Tform, Rmean, false);
        
        Tform_lidar_1_lidar_2 = [[Rmean,Tmean'];0,0,0,1];
        % disp('Guess: lidar_1 to lidar_2'); disp(Tform_lidar_1_lidar_2);
        % disp('GT: lidar_1 to lidar_2'); disp(Tform_gt_lidar_1_lidar_2);
        % disp('Evaulation: error');
        [e_x, e_y, e_z, e_roll, e_pitch, e_yaw] = ...
            calculate_error(Tform_gt_lidar_1_lidar_2, Tform_lidar_1_lidar_2);
        m_e_x(r,t) = e_x;
        m_e_y(r,t) = e_y;
        m_e_z(r,t) = e_z;
        m_e_roll(r,t) = e_roll;
        m_e_pitch(r,t) = e_pitch;
        m_e_yaw(r,t) = e_yaw;
        disp(r);
    end
    v_str_xlabel{t} = int2str(v_t(t));
    disp(['***************************', int2str(t)]);
end
figure; 
subplot(3,2,1); title('Rotation'); xlabel('t [s]'); ylabel('roll [rad]'); hold on; boxplot(m_e_roll, 'Labels', v_t);
subplot(3,2,3); xlabel('t [s]'); ylabel('pitch [rad]'); hold on; boxplot(m_e_pitch, 'Labels', v_t);
subplot(3,2,5); xlabel('t [s]'); ylabel('yaw [rad]'); hold on; boxplot(m_e_yaw, 'Labels', v_t);
subplot(3,2,2); title('Translation'); hold on; xlabel('t [s]'); ylabel('x [m]'); hold on; boxplot(m_e_x, 'Labels', v_t);
subplot(3,2,4); xlabel('t [s]'); ylabel('y [m]'); hold on; boxplot(m_e_y, 'Labels', v_t);
subplot(3,2,6); xlabel('t [s]'); ylabel('z [m]'); hold on; boxplot(m_e_z, 'Labels', v_t);
set(gca, 'FontSize', 10);

%% lidar_1 -> lidar_3
[m_e_x, m_e_y, m_e_z, m_e_roll, m_e_pitch, m_e_yaw] = deal(zeros(r_initialization, length(v_t)));
v_str_xlabel = cell(1,length(v_t));
disp('lidar_1 to lidar_3 ...');
for t = 1:length(v_t)
    for r = 1:r_initialization
        lidar_1_Tform = lidar_1_data.tformRel(:,:,1:v_t(t)*40);
        lidar_3_Tform = lidar_3_data.tformRel(:,:,1:v_t(t)*40);
        [Rmean, Rvar] = calcR(lidar_1_Tform, lidar_3_Tform);
        [Tmean, Tvar] = calcT(lidar_1_Tform, lidar_3_Tform, Rmean, false);
        
        Tform_lidar_1_lidar_3 = [[Rmean,Tmean'];0,0,0,1];
        disp('Guess: lidar_1 to lidar_2'); disp(Tform_lidar_1_lidar_3);
        disp('GT: lidar_1 to lidar_2'); disp(Tform_gt_lidar_1_lidar_3);
        % disp('Evaulation: error');
        [e_x, e_y, e_z, e_roll, e_pitch, e_yaw] = ...
            calculate_error(Tform_gt_lidar_1_lidar_3, Tform_lidar_1_lidar_3);
        m_e_x(r,t) = e_x;
        m_e_y(r,t) = e_y;
        m_e_z(r,t) = e_z;
        m_e_roll(r,t) = e_roll;
        m_e_pitch(r,t) = e_pitch;
        m_e_yaw(r,t) = e_yaw;
        disp(r);
    end
    v_str_xlabel{t} = int2str(v_t(t));
    disp(['***************************', int2str(t)]);
end
figure; 
subplot(3,2,1); title('Rotation'); xlabel('t [s]'); ylabel('roll [rad]'); hold on; boxplot(m_e_roll, 'Labels', v_t);
subplot(3,2,3); xlabel('t [s]'); ylabel('pitch [rad]'); hold on; boxplot(m_e_pitch, 'Labels', v_t);
subplot(3,2,5); xlabel('t [s]'); ylabel('yaw [rad]'); hold on; boxplot(m_e_yaw, 'Labels', v_t);
subplot(3,2,2); title('Translation'); hold on; xlabel('t [s]'); ylabel('x [m]'); hold on; boxplot(m_e_x, 'Labels', v_t);
subplot(3,2,4); xlabel('t [s]'); ylabel('y [m]'); hold on; boxplot(m_e_y, 'Labels', v_t);
subplot(3,2,6); xlabel('t [s]'); ylabel('z [m]'); hold on; boxplot(m_e_z, 'Labels', v_t);
set(gca, 'FontSize', 10);




