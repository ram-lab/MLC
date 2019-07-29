clc;
addpath('../../iv_2019');

sigma_r = sqrt(1e-4);
sigma_t = sqrt(1e-4);
fprintf('Simulation: sigma_r sigma_t:\n%f %f\n', sigma_r, sigma_t);

q_base = [0,0,0,1];
R_base = quat2rotm(q_base);
t_base = [-2, 0, 0]';
T_base = [R_base, t_base; 0 0 0 1];

% \nS.T 1
fprintf('\nS.T 1:**********************\n');
% Kabsch
q_measured = [0.0034    0.0011   -0.0001   -1.0000];
t_measured = [-1.6754    0.1013    0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0003    0.0020   -0.0032    1.0000];
t_measured = [-1.8498    0.0785         0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

% \nS.T 2
fprintf('\nS.T 2:**********************\n');
% Kabsch
q_measured = [0.0070   -0.0004   -0.0025    1.0000];
t_measured = [-1.8310    0.0795   0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0007   -0.0012    0.0002   -1.0000];
t_measured = [-1.8394    0.0352         0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

% \nS.T 3
fprintf('\nS.T 3:**********************\n');
% Kabsch
q_measured = [0.0002    0.1947    0.9809    0.0005];
t_measured = [-5.6817   -0.1169   0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0012    0.0036   -0.0005   -1.0000];
t_measured = [-1.8193    0.0513         0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

