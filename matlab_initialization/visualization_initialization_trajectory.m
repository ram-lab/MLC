clc; 
close all;
addpath('./yaml');
addpath('./kabsch');
addpath('./camodocal');

%% parameter setting
str_gt_mat = 'data/yellow_20180104';
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

top_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/eclipse/odom_top.bag';
front_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/eclipse/odom_front.bag';
tail_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/eclipse/odom_tail.bag'; 

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

%%
T_12 = [ ...
    0.99636, -0.055485, 0.064747, 0.098638; ...
    0.055005, 0.99844, 0.0091742, 0.34635; ...
    -0.065155, -0.0055794, 0.99786, -1.0054; ...
    0, 0, 0, 1.0000];
T_13 = [ ...
    -0.99437, 0.10598, -0.0006967, -0.33939;
    -0.10593, -0.99409, -0.023676, 1.262;
    -0.0032017, -0.023468, 0.99972, -1.1037;
    0, 0, 0, 1.0000];

for i = 1:length(lidar_1_data.tformAbs_world)
    tmp = lidar_1_data.tformAbs_world(1, 4, i);
    lidar_1_data.tformAbs_world(1, 4, i) = lidar_1_data.tformAbs_world(3, 4, i);
    lidar_1_data.tformAbs_world(3, 4, i) = lidar_1_data.tformAbs_world(2, 4, i);
    lidar_1_data.tformAbs_world(2, 4, i) = tmp;
end

for i = 1:length(lidar_2_data.tformAbs_world)
    tmp = lidar_2_data.tformAbs_world(1, 4, i);
    lidar_2_data.tformAbs_world(1, 4, i) = lidar_2_data.tformAbs_world(3, 4, i);
    lidar_2_data.tformAbs_world(3, 4, i) = lidar_2_data.tformAbs_world(2, 4, i);
    lidar_2_data.tformAbs_world(2, 4, i) = tmp;    
    lidar_2_data.tformAbs_world(:, :, i) = T_12 * lidar_2_data.tformAbs_world(:, :, i) * T_12^(-1);
end
for i = 1:length(lidar_3_data.tformAbs_world)
    tmp = lidar_3_data.tformAbs_world(1, 4, i);
    lidar_3_data.tformAbs_world(1, 4, i) = lidar_3_data.tformAbs_world(3, 4, i);
    lidar_3_data.tformAbs_world(3, 4, i) = lidar_3_data.tformAbs_world(2, 4, i);
    lidar_3_data.tformAbs_world(2, 4, i) = tmp;    
    lidar_3_data.tformAbs_world(:, :, i) = T_13 * lidar_3_data.tformAbs_world(:, :, i) * T_13^(-1);
end


%% draw now
figure;
for i=1:length(lidar_1_data.tformAbs_world(1, 4, :))
    x = reshape(lidar_1_data.tformAbs_world(1, 4, i), [1,length(lidar_1_data.tformAbs_world(1, 4, i))]);
    y = reshape(lidar_1_data.tformAbs_world(2, 4, i), [1,length(lidar_1_data.tformAbs_world(2, 4, i))]);
    z = reshape(lidar_1_data.tformAbs_world(3, 4, i), [1,length(lidar_1_data.tformAbs_world(2, 4, i))]);
    plot(x, y, 'k-', 'LineWidth', 3); 
    hold on;

    x = reshape(lidar_2_data.tformAbs_world(1, 4, i), [1,length(lidar_2_data.tformAbs_world(1, 4, i))]);
    y = reshape(lidar_2_data.tformAbs_world(2, 4, i), [1,length(lidar_2_data.tformAbs_world(2, 4, i))]);
    z = reshape(lidar_2_data.tformAbs_world(3, 4, i), [1,length(lidar_2_data.tformAbs_world(2, 4, i))]);
    plot(x, y, 'r-', 'LineWidth', 3); 
    hold on;

    x = reshape(lidar_3_data.tformAbs_world(1, 4, i), [1,length(lidar_3_data.tformAbs_world(1, 4, i))]);
    y = reshape(lidar_3_data.tformAbs_world(2, 4, i), [1,length(lidar_3_data.tformAbs_world(2, 4, i))]);
    z = reshape(lidar_3_data.tformAbs_world(3, 4, i), [1,length(lidar_3_data.tformAbs_world(2, 4, i))]);
    plot(x, y, 'b-', 'LineWidth', 3); 
    hold on;
    
    drawnow 
    axis equal; 
    xlabel('X (m)'); ylabel('Y (m)'); 
    set(gca, 'FontSize', 40);
    legend('\itl^{1}', '\itl^{2}', '\itl^{3}', 'FontSize', 40);
    grid on;
end


   
%% draw initialization
figure;
sensor_data = lidar_1_data;
x = reshape(sensor_data.tformAbs_world(1, 4, :), [1,length(sensor_data.tformAbs_world(1, 4, :))]);
y = reshape(sensor_data.tformAbs_world(2, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
z = reshape(sensor_data.tformAbs_world(3, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
plot(x, y, 'k-', 'LineWidth', 3); 
hold on;

sensor_data = lidar_2_data;
x = reshape(sensor_data.tformAbs_world(1, 4, :), [1,length(sensor_data.tformAbs_world(1, 4, :))]);
y = reshape(sensor_data.tformAbs_world(2, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
z = reshape(sensor_data.tformAbs_world(3, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
plot(x, y, 'r-', 'LineWidth', 3); 
hold on;

sensor_data = lidar_3_data;
x = reshape(sensor_data.tformAbs_world(1, 4, :), [1,length(sensor_data.tformAbs_world(1, 4, :))]);
y = reshape(sensor_data.tformAbs_world(2, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
z = reshape(sensor_data.tformAbs_world(3, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
plot(x, y, 'b-', 'LineWidth', 3); 
hold on;

axis equal; 
xlabel('X (m)'); ylabel('Y (m)'); 
set(gca, 'FontSize', 40);
legend('\itl^{1}', '\itl^{2}', '\itl^{3}', 'FontSize', 40);
grid on;

    
    
    
    