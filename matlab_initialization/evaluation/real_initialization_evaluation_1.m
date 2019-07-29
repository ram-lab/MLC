clc; close all;
addpath('../../iv_2019');

q_base = [0.001802, -0.043022, -0.0031786, 0.99907];
R_base = quat2rotm(q_base);
t_base = [-2.4104, 0.14744, -0.11854]';
T_base = [R_base, t_base; 0 0 0 1];

% R.T 1
fprintf('\nR.T 1:**********************\n');
% Kabsch
q_measured = [0.95562, 0.01185, 0.012358, 0.29411];
t_measured = [2.8059, -0.33781, -0.11854]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0015591, -0.012993, 0.016223, -0.99978];
t_measured = [-2.3213, -0.061759, -0.11854]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

% R.T 1
fprintf('\nR.T 1:**********************\n');
% Kabsch
q_measured = [0.14763     0.022594     0.014337     -0.98868];
t_measured = [-1.0341     -0.66132     -0.11854]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.033798      -0.0128    0.0058264     -0.99933];
t_measured = [-1.6543       1.3264 -0.11854]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)
