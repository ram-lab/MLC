function [X, x_kabsch, x_proposed] = initialization_planar(ref_data, target_data)

    l = min(ref_data.odom_range, target_data.odom_range);
    ini_X = eye(4,4);

    ref_data.tformRel_val = zeros(4, 4, 0); 
    target_data.tformRel_val = zeros(4, 4, 0);

    theta_error = zeros(l,1);
    dis_error = zeros(l,1);
    theta_epsilon = 0.01;
    dis_epsilon = 0.01;

    rotation_error_wo_calibration = zeros(l,1);
    translation_error_wo_calibration = zeros(l,1);
    rotation_error_calibration = zeros(l,1);
    translation_error_calibration = zeros(l,1);
    r_epsilon = 0.01;

    count_rotation_error = 0;
    count_rotation_threshold = 30;

    count_initilization = 0;
    count_optimization = 0;

    for i = 1:l
        % step 1: Wrong pair filtration based on screw motion theory
        % input: lidar_1_data, lidar_3_data
        % output: lidar_1_data.tformRel_val, lidar_3_data.tformRel_val, theta_error, dis_error
        if (i < 30) 
            continue;
        end
        lidar_1_Tform = ref_data.tformRel(:,:, i); 
        lidar_3_Tform = target_data.tformRel(:,:, i);
        [t_error, d_error] = motion_pair_filtration(lidar_1_Tform, lidar_3_Tform);
        theta_error(i) = t_error;
        dis_error(i) = d_error;

        if (t_error <= theta_epsilon) && (d_error <= dis_epsilon)   
            ref_data.tformRel_val = cat(3, ref_data.tformRel_val, lidar_1_Tform);
            target_data.tformRel_val = cat(3, target_data.tformRel_val, lidar_3_Tform);
        end    

        % step 2: Initialization
        if (i == l)
    %         if (strcmp(str_initializer, 'matrix'))
                % based on matrix: the Kabsch algorithm
                [Rmean, Rvar] = calcR(ref_data.tformRel_val, target_data.tformRel_val);    
                [Tmean, Tvar] = calcT(ref_data.tformRel_val, target_data.tformRel_val, Rmean, false);        
                ini_X = [[Rmean,Tmean'];0,0,0,1];
                ini_X(3, 4) = 0;

                x_kabsch = zeros(1, 7);
                x_kabsch(1:3) = ini_X(1:3, 4)';
                x_kabsch(4:7) = rotm2quat(ini_X(1:3,1:3));
                fprintf('Kabsch: tx ty tz qw qx qy qz\n'); 
                fprintf('%f %f %f %f %f %f %f\n', x_kabsch(1:7));

    %         elseif (strcmp(str_initializer, 'quaternion'))
                % based on quaternion-optimization: minimize ||Aq||, st.||q||=1
                % planner movement
                [b_solve, b_rot_con, R_yx]  = estimate_Ryx(ref_data.tformRel_val, target_data.tformRel_val); % R_yx = eul2rotm([pi 0 0], 'ZYX');
                [R_z, t_x, t_y] = estimate_Rz_tx_ty(ref_data.tformRel_val, target_data.tformRel_val, R_yx);
                ini_R = R_z * R_yx;
                ini_t = [t_x; t_y; 0]; % z-translation is unknown
                ini_X = [[ini_R,ini_t];0,0,0,1];

                x_proposed = zeros(1, 7);
                x_proposed(1:3) = ini_X(1:3, 4)';
                x_proposed(4:7) = rotm2quat(ini_X(1:3,1:3));
                fprintf('Proposed: tx ty tz qw qx qy qz\n'); 
                fprintf('%f %f %f %f %f %f %f\n', x_proposed(1:7));
    %         end

            count_initilization = count_initilization + 1;
            count_rotation_error = 0;        
        end  
    end
    X = ini_X;    
    
    %% visualize screw motion residual
%     figure;
%     subplot(1,2,1); title(''); hold on; plot(1:1:l, theta_error, '-', 'Color', [228, 26, 28]/255); hold off;
%     set(gca, 'FontName', 'Times New Roman', 'FontSize', 40);
%     set(gca,'position',[0.12,0.2,0.35,0.45] );
%     xlabel('Timestamp');
%     ylabel('Rotation residual [rad]');
%     ax = gca;
%     ax.LineWidth=4;    
%     grid on;
% 
% 
%     subplot(1,2,2); title(''); hold on; plot(1:1:l, dis_error, '-',  'Color', [228, 26, 28]/255); hold off;
%     set(gca, 'FontName', 'Times New Roman', 'FontSize', 40);
%     set(gca,'position',[0.61,0.2,0.35,0.45] );
%     xlabel('Timestamp');
%     ylabel('Translation residual [m]');
%     ax = gca;
%     ax.LineWidth=4;
%     grid on;

    close all;    
    
end