%clc, 
close all;

addpath('/home/jjiao/Documents/matlab_ws/Making-elegant-Matlab-figures/Boxplot');
%%
config = {'top_front_estimated.txt', 'top_tail_estimated.txt'}; % j
trajectory = {'8_shape', 'eclipse'}; % j
T_gt_12 = [ ...
      0.99612    -0.033733     0.081251      0.42039;...
     0.034646      0.99935   -0.0098511  -0.00066243;...
    -0.080866     0.012628      0.99664      -1.2597;...
            0            0            0            1];  
% T_gt_13 = [ ...
%      -0.99954     0.029889   -0.0044847      -1.9873;...
%     -0.029799     -0.99938    -0.018984     0.063455;...
%    -0.0050493    -0.018841      0.99981      -1.1817;...
%             0            0            0            1];    
T_gt_13 = [ ...
     -0.99954     0.029889   -0.0044847      -2.1;...
    -0.029799     -0.99938    -0.018984     0.063455;...
   -0.0050493    -0.018841      0.99981      -1.1817;...
            0            0            0            1];   
rotation_error = cell(2,2);
translation_error = cell(2,2);
reg_error = cell(2,2);
result = cell(2, 2);

%%
for i = 1:2
    for j = 1:2
        filename = ['/home/jjiao/Documents/matlab_ws/matlab_multi_lidar_cal/iros_2019/data/refinement_data/0228_2/',trajectory{i},'/',config{j}];
        file = strcat(filename);
        [data] = textread(file);        
        v_x = zeros(length(data), 7);
        v_reg_error = zeros(length(data), 1);
        for k = 1:length(data)
            x(k, :) = data(k, 2:8);
            v_reg_error(k, 1) = data(k, 11);
        end
        rotation_error{j, i} = zeros(length(v_reg_error), 1);
        translation_error{j, i} = zeros(length(v_reg_error), 1);
        result{j, i} = zeros(length(v_reg_error), 7);
        
        if mod(j, 2) % top_front
            for k = 1:length(v_reg_error)
                T_measured = [quat2rotm(x(k,4:7)), x(k, 1:3)'; 0 0 0 1]; 
                [r_error, t_error] = extrinsic_error(T_gt_12, T_measured);
                rotation_error{j, i}(k, 1) = r_error;
                translation_error{j, i}(k, 1) = t_error;
                reg_error{j, i}(k, 1) = v_reg_error(k, 1);
                result{j, i}(k, :) = x(k, :);
            end
        else % top_tail
            for k = 1:length(v_reg_error)
                T_measured = [quat2rotm(x(k,4:7)), x(k, 1:3)'; 0 0 0 1]; 
                [r_error, t_error] = extrinsic_error(T_gt_13, T_measured);
                rotation_error{j, i}(k, 1) = r_error;
                translation_error{j, i}(k, 1) = t_error;
                reg_error{j, i}(k, 1) = v_reg_error(k, 1);      
                result{j, i}(k, :) = x(k, :);
            end            
        end
    end
end

filter_result = cell(2, 2);
filter_result_xyz_ypr = cell(2, 2);
filter_rotation_error = cell(2,2);
filter_translation_error = cell(2,2);
filter_reg_error = cell(2,2);
c = 1;
K = 10;
% figure;
for i = 1:2
    for j = 1:2
        disp([trajectory(i), ' ', config(j)]);
        
%         subplot(2,2,c); c = c+1;
%         plot(1:1:length(rotation_error{j, i}), rotation_error{j, i}, 'b');
%         title('Rotation Error'); xlabel('Timestamp'); ylabel('Error [rad]');
%         hold on;
% 
%         % figure;
%         plot(1:1:length(translation_error{j, i}), translation_error{j, i}, 'k');
%         title('Translation Error'); xlabel('Timestamp'); ylabel('Error');
%         hold on;
% 
%         % figure;
%         plot(1:1:length(reg_error{j, i}), reg_error{j, i}/100, 'r');
%         title('Translation Error'); xlabel('Timestamp'); ylabel('Error');
%         hold off;
        
        [min_reg, min_i] = mink(reg_error{j, i}, K);
        filter_result{j, i} = zeros(K, 7);
        filter_result_xyz_ypr{j, i} = zeros(K, 6);
        filter_rotation_error{j, i} = zeros(K, 1);
        filter_translation_error{j, i} = zeros(K, 1);
        filter_reg_error{j, i} = zeros(K, 1);
        
        for l = 1:length(min_i)
            filter_result{j, i}(l, :) = result{j, i}(min_i(l), :);
            x = filter_result{j, i}(l, :);
            filter_result_xyz_ypr{j, i}(l, 1:3) = x(1:3);
            filter_result_xyz_ypr{j, i}(l, 4:6) = quat2eul(x(4:7), 'ZYX');            
            
            filter_rotation_error{j, i}(l, 1) = rotation_error{j, i}(min_i(l), :);
            filter_translation_error{j, i}(l, 1) = translation_error{j, i}(min_i(l), :);
            filter_reg_error{j, i}(l, 1) = reg_error{j, i}(min_i(l), :);
%             fprintf("Error: %f %f\n", filter_rotation_error{j, i}(l, 1), filter_translation_error{j, i}(l, 1));
        end
    end
end

%% plot 
fig = figure(2);
set(fig, 'defaultAxesColorOrder', [228/255 26/255 28/255; 9/255 59/255 170/255]);
% set(fig, 'defaultAxesColorOrder', [0/255 0/255 0/255; 0/255 0/255 0/255]);
fig = subplot(121);
subplot('Position',[0.01,0.40,0.45,0.5])
grid on;

yyaxis left;
plot(1:1:length(rotation_error{2, 1}), rotation_error{2, 1}, 'Color', [228, 26, 28]/255, 'linewidth',1);
% title('Rotation Error'); 
xlabel('Timestamp'); ylabel('Rotation Error [rad]');
ylim([0, 1.5]);
yyaxis right;
plot(1:1:length(reg_error{2, 1}), reg_error{2, 1}, 'Color', [9, 59, 170]/255, 'linewidth',1);
ylabel('ICP Error');
ylim([0, 1700]);
hold off;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 40, 'linewidth',4);
axis square;

% fig = figure;
fig = figure(2);
set(fig, 'defaultAxesColorOrder', [228/255 26/255 28/255; 9/255 59/255 170/255]);
% set(fig, 'defaultAxesColorOrder', [0/255 0/255 0/255; 0/255 0/255 0/255]);
fig = subplot(122);
subplot('Position',[0.52,0.40,0.45,0.5])
grid on;

yyaxis left;
plot(1:1:length(translation_error{2, 1}), translation_error{2, 1}, 'Color', [228, 26, 28]/255, 'linewidth',1);
% title('Translation Error'); 
ylim([0, 15]);
xlabel('Timestamp'); ylabel('Translation Error [m]');
yyaxis right;
plot(1:1:length(reg_error{2, 1}), reg_error{2, 1}, 'Color', [9, 59, 170]/255, 'linewidth',1);
ylabel('ICP Error');
ylim([0, 1700]);
hold off;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 40, 'linewidth',4);

sgtitle('The Calibration Error (Rotation and Translation ) and ICP Error', 'FontName', 'Times New Roman', 'FontSize', 40);
axis square;


%%
mean_pose = cell(2, 2);
for i = 1:2
    for j = 1:2
        mean_pose{j, i} = zeros(1, 7);
        mean_t = mean(filter_result{j, i}(:, 1:3), 1);
        mean_quat = meanrot(quaternion(filter_result{j, i}(:, 4:7)));
        mean_euler = eulerd(mean_quat,'ZYX','frame');
        mean_pose{j, i}(1, 1:3) = mean_t;
        mean_pose{j, i}(1, 4:6) = mean_euler/180*pi;

        t = mean_t;
        q = eul2quat(mean_euler/180*pi, 'ZYX');
        T_measured = [quat2rotm(q), mean_t'; 0 0 0 1];       
        if mod(j, 2)
            [r_error, t_error] = extrinsic_error(T_gt_12, T_measured);        
        else
            [r_error, t_error] = extrinsic_error(T_gt_13, T_measured);
        end
        fprintf("Error: e_r:%f e_t:%f\n", r_error, t_error);        
    end
end

num_boxes = 4;    
order = cell(2, 2); order{1,1} = 1; order{2,1} = 3; order{1,2} = 2; order{2,2} = 4;
et_data = cell(1,num_boxes);   
er_data = cell(1,num_boxes);   
for i = 1:2
    for j = 1:2
        er_data{1, order{j, i}} = filter_rotation_error{j, i}(:, 1);        
        et_data{1, order{j, i}} = filter_translation_error{j, i}(:, 1);
    end
end
figure;
subplot(121);
subplot('Position',[0.05,0.40,0.37,0.5])
label_axes = {'Trajectory','Rotation Error [rad]'}; 
label_boxes = {'R.T 1','R.T 1','R.T 2','R.T 2'};
box_color = [[228, 26, 28]/255; [9, 59, 170]/255; [228, 26, 28]/255; [9, 59, 170]/255];
figure_boxplot(er_data,label_axes,label_boxes, '', 'horizontal', box_color);
legend('$l^{1}\ominus l^{2}$', '$l^{1}\ominus l^{3}$', 'Interpreter','latex', 'FontSize', 40);
grid on;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 30);
ax = gca;
ax.LineWidth = 4;


subplot(122);
subplot('Position',[0.57,0.40,0.37,0.5])
label_axes = {'Trajectory','Translation Error [m]'}; 
label_boxes = {'R.T 1','R.T 1','R.T 2','R.T 2'};
box_color = [[228, 26, 28]/255; [9, 59, 170]/255; [228, 26, 28]/255; [9, 59, 170]/255];
figure_boxplot(et_data,label_axes,label_boxes, '', 'horizontal', box_color);
legend('$l^{1}\ominus l^{2}$', '$l^{1}\ominus l^{3}$', 'Interpreter','latex', 'FontSize', 40);
grid on;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 30);
ax = gca;
ax.LineWidth = 4;

sgtitle('Rotation and Translation Error of the Selected Transformations', 'FontName', 'Times New Roman', 'FontSize', 40);





% subplot('Position',[left bottom width height])




