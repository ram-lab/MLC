clc; close all;
addpath('./yaml');
addpath('./kabsch');

%% parameter setting
str_gt_mat = 'data/simulation_carla_gt';
str_odom_obtain = 'data/simulation_odom_2';
str_odom_bag = 'read_from_bag'; % 'calculate_from_pcd'
str_initializer = 'quaternion';
b_odom_obtain = false;
b_abs_rel_odom_check = false;
b_plot_odom = false;
b_noise = false;

disp('Load ground truth ...');
load(str_gt_mat);

rng(2);

%% obtain odom
if b_odom_obtain
    disp('Load odom ...');
    load(str_odom_obtain);
else
    if strcmp(str_odom_bag, 'read_from_bag')
        disp('Read odom from bag ...');        
        bag_file = '/Monster/dataset/lidar_calibration/yellow_lidar_calibration_20181124/odom/loam_top.bag';
        lidar_1_data = odom_obtain_from_bag(bag_file, 'Transformation of top lidar', 1, '/odom_from_start', b_plot_odom, b_noise);
        bag_file = '/Monster/dataset/lidar_calibration/yellow_lidar_calibration_20181124/odom/loam_front.bag';
        lidar_2_data = odom_obtain_from_bag(bag_file, 'Transformation of front lidar', 2, '/odom_from_start', b_plot_odom, b_noise);
        bag_file = '/Monster/dataset/lidar_calibration/yellow_lidar_calibration_20181124/odom/loam_tail.bag';
        lidar_3_data = odom_obtain_from_bag(bag_file, 'Transformation of tail lidar', 3, '/odom_from_start', b_plot_odom, b_noise);
%         bag_file = '/Monster/dataset/lidar_calibration/carla_lidar_calibration_20181217/episode_0_carla_multilidar_4.bag';
%         lidar_1_data = odom_obtain_from_bag(bag_file, 'Transformation of top lidar', 1, '/odom_from_top_lidar', b_plot_odom, b_noise);
%         bag_file = '/Monster/dataset/lidar_calibration/carla_lidar_calibration_20181217/episode_0_carla_multilidar_4.bag';
%         lidar_2_data = odom_obtain_from_bag(bag_file, 'Transformation of front lidar', 2, '/odom_from_top_lidar', b_plot_odom, b_noise);
%         bag_file = '/Monster/dataset/lidar_calibration/carla_lidar_calibration_20181217/episode_0_carla_multilidar_4.bag';
%         lidar_3_data = odom_obtain_from_bag(bag_file, 'Transformation of tail lidar', 3, '/odom_from_tail_lidar', b_plot_odom, b_noise);
    elseif strcmp(str_odom_bag, 'calculate_from_pcd')
        fprintf('Calculate odom from pcd ...');
        data_path = '/Monster/dataset/lidar_calibration/yellow_lidar_calibration_20181124/pcd/';
        lidar_1_data = gen_lidar_tf([data_path, 'top/'], true, 'TOP');
        lidar_2_data = gen_lidar_tf([data_path, 'front/'], true, 'FRONT');
        lidar_3_data = gen_lidar_tf([data_path, 'tail/'], true, 'TAIL');
    end
end
figure;
plot_sensor_odom(lidar_1_data, 'Trajectory in top lidar', 3, 1);
plot_sensor_odom(lidar_2_data, 'Trajectory in front lidar', 3, 2);
plot_sensor_odom(lidar_3_data, 'Trajectory in tail lidar', 3, 3);

if b_abs_rel_odom_check
    disp(lidar_1_data.tformAbs_lidar(:,:,end));
    tform = eye(4);
    for frame = 1:length(lidar_1_data.tformRel)
        tform = tform * lidar_1_data.tformRel(:,:,frame);
    end
    disp(tform); disp('checking if equal ?');
end

disp('Finish odom acquirement !');

%% lidar_1 -> lidar_3
disp('lidar_1 to lidar_3 ...');
l = min(lidar_1_data.odom_range, lidar_3_data.odom_range);
% X = Tform_gt_lidar_1_lidar_3;
X = eye(4,4);

lidar_1_data.tformRel_val = zeros(4, 4, 0); 
lidar_3_data.tformRel_val = zeros(4, 4, 0);

theta_error = zeros(l,1);
dis_error = zeros(l,1);
theta_epsilon = 0.01;
dis_epsilon = 0.1;

rotation_error = zeros(l,1);
translation_error = zeros(l,1);
r_epsilon = 0.01;

count_rotation_error = 0;
count_rotation_threshold = 30;

count_initilization = 0;
count_optimization = 0;

%% Online calibration
for i = 1:l
    % step 1: Wrong pair filtration based on screw motion theory
    % input: lidar_1_data, lidar_3_data
    % output: lidar_1_Tform, lidar_3_Tform, theta_error, dis_error
    lidar_1_Tform = lidar_1_data.tformRel(:,:, i); 
    lidar_3_Tform = lidar_3_data.tformRel(:,:, i);
    [t_error, d_error] = motion_pair_filtration(lidar_1_Tform, lidar_3_Tform);
    theta_error(i) = t_error;
    dis_error(i) = d_error;
    
    if (t_error <= theta_epsilon) && (d_error <= dis_epsilon)   
        lidar_1_data.tformRel_val = cat(3, lidar_1_data.tformRel_val, lidar_1_Tform);
        lidar_3_data.tformRel_val = cat(3, lidar_3_data.tformRel_val, lidar_3_Tform);
        % step 2: Miscalibration detection
        [r_error, t_error] = miscalibration_error(lidar_1_Tform, lidar_3_Tform, X,'angle');
        rotation_error(i) = r_error;
        translation_error(i) = t_error;       
        if (r_error > r_epsilon)
            count_rotation_error = count_rotation_error + 1;
        end
    end    
    
    if (count_rotation_error > count_rotation_threshold) || (i == l)
        if (strcmp(str_initializer, 'matrix'))
            % step 3: Initializer(based on matrix)            
            [Rmean, Rvar] = calcR(lidar_1_data.tformRel_val, lidar_3_data.tformRel_val);    
            [Tmean, Tvar] = calcT(lidar_1_data.tformRel_val, lidar_3_data.tformRel_val, Rmean, false);        
            ini_X = [[Rmean,Tmean'];0,0,0,1];
            rotm2eul(ini_X(1:3,1:3))/pi*180        
            disp('Number of poses'); disp(length(lidar_1_data.tformRel_val));
            disp('start index'); disp(i);
            disp('Guess: lidar_1 to lidar_3'); disp(ini_X);
            disp('GT: lidar_1 to lidar_2'); disp(Tform_gt_lidar_1_lidar_3);
            disp('Count initializaion'); disp(count_initilization);
            
        elseif (strcmp(str_initializer, 'quaternion'))
            % step 3: Initializer(based on quaternion)
            % based on quaternion-optimization: minimize ||Aq||, st.||q||=1
            % non-planner movement  
            q_zyx = estimate_qzyx(lidar_1_data.tformRel_val, lidar_3_data.tformRel_val);
            R_sol = quat2rotm(q_zyx);
%             R_sol = Tform_gt_lidar_1_lidar_3(1:3,1:3);
            [Tmean, Tvar] = calcT(lidar_1_data.tformRel_val, lidar_3_data.tformRel_val, R_sol, false);        
            ini_X = [[R_sol,Tmean'];0,0,0,1];
            rotm2eul(ini_X(1:3,1:3))/pi*180        
            disp('Number of poses'); disp(length(lidar_1_data.tformRel_val));
            disp('start index'); disp(i);
            disp('Guess: lidar_1 to lidar_3'); disp(ini_X);
            disp('GT: lidar_1 to lidar_3'); disp(Tform_gt_lidar_1_lidar_3);         
        end
        disp('Count initializaion'); disp(count_initilization);         
        count_initilization = count_initilization + 1;
        count_rotation_error = 0;        
    end 
    
    % step 4: Optimization, model uncertainty
    
    
end

figure;
subplot(2,1,1); title('invariant(rotation)'); hold on; plot(1:1:l, theta_error, 'r-'); hold off;
subplot(2,1,2); title('invariant(translation)'); hold on; plot(1:1:l, dis_error, 'b-'); hold off;

figure;
subplot(2,1,1); title('rotation error(angle)'); hold on; plot(1:1:l, rotation_error, 'r-'); hold off;
subplot(2,1,2); title('translation error'); hold on; plot(1:1:l, translation_error, 'b-'); hold off;
















