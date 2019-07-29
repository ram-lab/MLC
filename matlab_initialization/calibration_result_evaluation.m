clc;
%% yellow (top-front)
disp('yellow top-front ------------------------');

% hand-labelling
q_base = [0.9985353, 0.0058359, 0.0493219, 0.021461];
R_base = quat2rotm(q_base);
t_base = [0.377002, -0.0309009, -1.23236]' * 100;
T_base = [R_base, t_base; 0 0 0 1];

% without refinement
disp('without refinement:**********************');
R_measured = [0.99335   0.0198619   -0.113405;...
                -0.0192156    0.999792  0.00678938;...
                0.113516 -0.00456509    0.993526];
q_measured = rotm2quat(R_measured);
t_measured = [0.314979 0.0509327  -1.23395]' * 100;
T_measured = [R_measured, t_measured; 0 0 0 1];
disp('rotation error, translation error :');
[r_error, t_error] = extrinsic_error(T_base, T_measured)

% icp refinement
disp('icp refinement:**********************');
R_measured = [0.909913, -0.00973191,   -0.414685;
                0.0421126,    0.996727,   0.0690134;
                0.412656,  -0.0802597,    0.907344];
q_measured = rotm2quat(R_measured);
t_measured = [1.21884 -0.0670357 -1.55964]' * 100;
T_measured = [R_measured, t_measured; 0 0 0 1];
disp('rotation error, translation error :');
[r_error, t_error] = extrinsic_error(T_base, T_measured)

% nonlinear refinement
disp('nonlinear refinement:**********************');
q_measured = [0.998784 0.00121095 0.0445531 0.013818];
R_measured = quat2rotm(q_measured);
t_measured = [0.336322 0.00271319 -1.19076]' * 100;
T_measured = [R_measured, t_measured; 0 0 0 1];
disp('rotation error, translation error:');
[r_error, t_error] = extrinsic_error(T_base, T_measured)



%% yellow (top-tail)
disp('yellow top-tail ------------------------');

% hand-labelling
q_base = [0.0012859 0.0095936 -0.0080411 0.9999208];
R_base = quat2rotm(q_base);
t_base = [-1.96443 0.0473154 -1.13756]' * 100;
T_base = [R_base, t_base; 0 0 0 1];

% without refinement
disp('without refinement:**********************');
R_measured = [-0.999736 -0.00994325   0.0207151;...
                0.00883716   -0.998565   -0.052819;...
                0.0212106   -0.052622    0.998389];
q_measured = rotm2quat(R_measured);
t_measured = [-1.92115 0.0968707  -0.80199]' * 100;
T_measured = [R_measured, t_measured; 0 0 0 1];
disp('rotation error, translation error :');
[r_error, t_error] = extrinsic_error(T_base, T_measured)

% icp refinement
disp('icp refinement:**********************');
R_measured = [-0.99108, 0.00944037,  -0.132932;
                -0.013735,  -0.999412,  0.0314269;
                -0.132557,  0.0329725,   0.990626];
q_measured = rotm2quat(R_measured);
t_measured = [-1.95929 0.0473068 -0.383619]' * 100;
T_measured = [R_measured, t_measured; 0 0 0 1];
disp('rotation error, translation error :');
[r_error, t_error] = extrinsic_error(T_base, T_measured)

% nonlinear refinement
disp('nonlinear refinement:**********************');
q_measured = [0.0019 0.0010 0.0002 1.0000];
R_measured = quat2rotm(q_measured);
t_measured = [-1.91 0.05299 -1.0744]' * 100;
T_measured = [R_measured, t_measured; 0 0 0 1];
disp('rotation error, translation error :');
[r_error, t_error] = extrinsic_error(T_base, T_measured)
