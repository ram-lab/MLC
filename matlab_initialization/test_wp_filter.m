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
disp('Finish odom acquirement !');

%% Wrong pair filtration based on screw motion theory
%% lidar_1 -> lidar_3
disp('lidar_1 to lidar_3 ...');
l = min(length(lidar_1_data.tformRel), length(lidar_3_data.tformRel));
lidar_1_Tform = lidar_1_data.tformRel(:,:,1:l); 
lidar_3_Tform = lidar_3_data.tformRel(:,:,1:l);
X = Tform_gt_lidar_1_lidar_3;
% X = eye(4,4);

theta_error = zeros(l,1);
rotation_error = zeros(l,1);
translation_error = zeros(l,1);
rotation_error_2 = zeros(l,1);
translation_error_2 = zeros(l,1);
d_error = zeros(l,1);
for i = 1:l
    A = lidar_1_Tform(:,:,i);
    B = lidar_3_Tform(:,:,i);
    R = X(1:3, 1:3);
    t = X(1:3, 4);
    
    % motion pair error verification
    r_A = vrrotmat2vec(A(1:3,1:3));
    r_B = vrrotmat2vec(B(1:3,1:3));
    theta_A = r_A(4);
    theta_B = r_B(4);
    r_A = r_A(1:3)';
    r_B = r_B(1:3)';
    theta_error(i) = abs(theta_A-theta_B);
    d_A = dot(A(1:3,4), r_A);
    d_B = dot(B(1:3,4), r_B);
    d_error(i) = abs(d_A-d_B);
    
    % miccalibration detection 1
    r_A = vrrotmat2vec(A(1:3,1:3));
    r_B = vrrotmat2vec(B(1:3,1:3));
    if abs(r_A(4)) < 0.0005
        r_A = zeros(3,1);
    else
        r_A = r_A(1:3)';
    end
    if abs(r_B(4)) < 0.0005
        r_B = zeros(3,1);
    else
        r_B = r_B(1:3)';
    end    
    r_error = r_A - R*r_B;
    rotation_error(i) = norm(r_error, 2); % |RrB-rA|, length
    
    t_error = (A(1:3,1:3) - eye(3))*t + A(1:3,4) - R*B(1:3,4); % |(RA-I)*t+tA-RtB|
    translation_error(i) = norm(t_error, 2); % use the translation_error to detect miscalibration
    
    % miccalibration detection 2
    r = vrrotmat2vec(A(1:3,1:3)*X(1:3,1:3) * (X(1:3,1:3)*B(1:3,1:3))^(-1));
    r = r * r(4);
    rotation_error_2(i) = norm(r(1:3), 2); % |log(AX * (XB)^(-1))|, rad
    
    t_error = (A(1:3,1:3) - eye(3))*t + A(1:3,4) - R*B(1:3,4); % |(RA-I)*t+tA-RtB|
    translation_error_2(i) = norm(t_error, 2); % use the translation_error to detect miscalibration    
end

count_rotation_error = sum(rotation_error < 0.1);
count_translation_error = sum(translation_error < 0.1);

figure;
subplot(2,1,1); title('rotation error(vector)'); hold on; plot(1:1:l, rotation_error, 'r-'); hold off;
subplot(2,1,2); title('translation error'); hold on; plot(1:1:l, translation_error, 'b-'); hold off;

figure;
subplot(2,1,1); title('rotation error(angle)'); hold on; plot(1:1:l, rotation_error_2, 'r-'); hold off;
subplot(2,1,2); title('translation error'); hold on; plot(1:1:l, translation_error_2, 'b-'); hold off;















