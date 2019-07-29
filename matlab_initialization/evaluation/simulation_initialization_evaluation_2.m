clc;
addpath('../../iv_2019');

sigma_r = sqrt(1e-3);
sigma_t = sqrt(1e-3);
fprintf('Simulation: sigma_r sigma_t: %f %f', sigma_r, sigma_t);

q_base = [0,0,0,1];
R_base = quat2rotm(q_base);
t_base = [-2, 0, 0]';
T_base = [R_base, t_base; 0 0 0 1];

% S.T 1
fprintf('\nS.T 1:**********************\n');
% Kabsch
q_measured = [0.0023   -0.0000    0.0000    1.0000];
t_measured = [-0.6768    0.1491    0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0080    0.0018   -0.0002    1.0000];
t_measured = [-1.5847    0.2799         0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

% S.T 2
fprintf('\nS.T 2:**********************\n');
% Kabsch
q_measured = [0.0301   -0.0115   -0.0054    0.9995];
t_measured = [-0.6464    0.1804   0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0027    0.0025    0.0020    1.0000];
t_measured = [-1.2121    0.1941         0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

% S.T 3
fprintf('\nS.T 3:**********************\n');
% Kabsch
q_measured = [0.0000   -0.5941    0.8044   -0.0002];
t_measured = [9.1693    0.6763   0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('Kabsch error:\n%f %f\n', r_error, t_error)
% Proposed
q_measured = [0.0892   -0.5399    0.1338    0.8262];
t_measured = [-1.8439   -0.0080         0]';
T_measured = [quat2rotm(q_measured), t_measured; 0 0 0 1];
[r_error, t_error] = extrinsic_error(T_base, T_measured);
fprintf('qw qx qy qz tx ty tz:\n%f %f %f %f %f %f %f\n', q_measured(1), q_measured(2), q_measured(3), q_measured(4), t_measured(1), t_measured(2), t_measured(3))
fprintf('proposed error:\n%f %f\n', r_error, t_error)

