clc, clear;
addpath('../../iv_2019');

fprintf('Real Sensor:\n');

load('../data/initialization/simulate_gt.mat');

%% noise 1
load('../data/initialization/simulate_2_noise_1.mat');
fprintf('GT noise 1: \n');
fprintf('tx ty tz qw qx qy qz:\n%f %f %f %f %f %f %f\n', x_gt_12(1:7));
T_gt_12 = [quat2rotm(x_gt_12(4:7)), x_gt_12(1:3)'; 0 0 0 1];

fprintf('Kabsch: \n');
x_kabsch_12(3) = x_gt_12(3);
fprintf('tx ty tz qw qx qy qz:\n%f %f %f %f %f %f %f\n', x_kabsch_12(1:7)); 
T_measured = [quat2rotm(x_kabsch_12(4:7)), x_kabsch_12(1:3)'; 0 0 0 1]; 
[r_error, t_error] = extrinsic_error(T_gt_12, T_measured);
fprintf('Error: %f %f\n', r_error, t_error);

fprintf('Proposed: \n');
x_proposed_12(3) = x_gt_12(3);
fprintf('tx ty tz qw qx qy qz:\n%f %f %f %f %f %f %f\n', x_proposed_12(1:7)); 
T_measured = [quat2rotm(x_proposed_12(4:7)), x_proposed_12(1:3)'; 0 0 0 1]; 
[r_error, t_error] = extrinsic_error(T_gt_12, T_measured);
fprintf('Error: %f %f\n\n', r_error, t_error);

%% noise 2
load('../data/initialization/simulate_2_noise_2.mat');
fprintf('GT noise 2: \n');
fprintf('tx ty tz qw qx qy qz:\n%f %f %f %f %f %f %f\n', x_gt_12(1:7));
T_gt_12 = [quat2rotm(x_gt_12(4:7)), x_gt_12(1:3)'; 0 0 0 1];

fprintf('Kabsch: \n');
x_kabsch_12(3) = x_gt_12(3);
fprintf('tx ty tz qw qx qy qz:\n%f %f %f %f %f %f %f\n', x_kabsch_12(1:7)); 
T_measured = [quat2rotm(x_kabsch_12(4:7)), x_kabsch_12(1:3)'; 0 0 0 1]; 
[r_error, t_error] = extrinsic_error(T_gt_12, T_measured);
fprintf('Error: %f %f\n', r_error, t_error);

fprintf('Proposed: \n');
x_proposed_12(3) = x_gt_12(3);
fprintf('tx ty tz qw qx qy qz:\n%f %f %f %f %f %f %f\n', x_proposed_12(1:7)); 
T_measured = [quat2rotm(x_proposed_12(4:7)), x_proposed_12(1:3)'; 0 0 0 1]; 
[r_error, t_error] = extrinsic_error(T_gt_12, T_measured);
fprintf('Error: %f %f\n\n', r_error, t_error);



