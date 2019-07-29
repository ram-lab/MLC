clc; 
close all;
addpath('../');

bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_tail.bag';
sensor_data = odom_obtain_from_bag(bag_file, 'Transformation of top lidar', 1, '/odom_from_start', false, false);        
x = reshape(sensor_data.tformAbs_lidar(1, 4, :), [1,length(sensor_data.tformAbs_lidar(1, 4, :))]);
y = reshape(sensor_data.tformAbs_lidar(2, 4, :), [1,length(sensor_data.tformAbs_lidar(2, 4, :))]);            
z = reshape(sensor_data.tformAbs_lidar(3, 4, :), [1,length(sensor_data.tformAbs_lidar(2, 4, :))]);                
figure;
plot(x, y, '-', 'Color', [190,10,50]/255, 'LineWidth', 5); 
title('Estimated trajectory of l_3 at R.T 1'); 
% axis sqare; 
% axis off;
hold on;
plot(0, 0, 'ko','MarkerSize', 10, 'MarkerFaceColor', 'k');    
xlabel('X [m]'); ylabel('Y [m]'); 
ax = gca;
set(ax, 'FontName', 'Times New Roman', 'FontSize', 35);
% set(gca,'position',[0.1,0.15,0.65,0.8] );
grid on;        
xlim([-15, 5]);
ylim([-35, 5]);
% legend('R.T 1', 'Start Location', 'Location', 'northwest');

figure;
bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/eclipse/odom_top.bag';
sensor_data = odom_obtain_from_bag(bag_file, 'Transformation of top lidar', 1, '/odom_from_start', false, false);        
x = reshape(sensor_data.tformAbs_lidar(1, 4, :), [1,length(sensor_data.tfoXmAbs_lidar(1, 4, :))]);
y = reshape(sensor_data.tformAbs_lidar(2, 4, :), [1,length(sensor_data.tformAbs_lidar(2, 4, :))]);            
z = reshape(sensor_data.tformAbs_lidar(3, 4, :), [1,length(sensor_data.tformAbs_lidar(2, 4, :))]);                
plot(x, y, '-', 'Color', [0, 0, 153]/255, 'LineWidth', 10); 
% axis sqare; 
axis off;
hold on;
xlim([-10, 10]);
ylim([-2, 31]);
% plot(0, 0, 'ko','MarkerSize', 20, 'MarkerFaceColor', 'k');    
xlabel('X [m]'); ylabel('Y [m]'); 
ax = gca;
set(ax, 'FontName', 'Times New Roman', 'FontSize', 35);
% set(gca,'position',[0.1,0.15,0.65,0.8] );
% legend('R.T 2', 'Start Location', 'Location', 'northwest');
grid on;

% [0, 0, 153]
% [190,10,50]
% [254,178,76]