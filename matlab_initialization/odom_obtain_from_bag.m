function [ lidar_data ] = odom_obtain_from_bag(bag_file, comment, k, topic, plot_odom, b_noise)
    % noise setup
    sigma_r = sqrt(1e-4);
    sigma_t = sqrt(1e-4);

    % setup
    lidar_data.bag_file = bag_file;
    lidar_data.bag_select = rosbag(bag_file);

    topic_select = select(lidar_data.bag_select, 'Topic', topic);
    odom_msg = readMessages(topic_select, 'DataFormat', 'struct');

    % filter unnecessary odom
%     if strcmp(topic, '/front/rslidar_points')
%         odom_msg = odom_msg(2:500);
%     else
%         odom_msg = odom_msg(1:500);
%     end

    lidar_data.odom_range = length(odom_msg);
    lidar_data.tformAbs_world = zeros(4, 4, lidar_data.odom_range); % trajectory in world coordinate system
    lidar_data.tformAbs_world(:,:,1) = eye(4);

    lidar_data.tformAbs_lidar = zeros(4, 4, lidar_data.odom_range); % trajectory in lidar coordinate system
    lidar_data.tformAbs_lidar(:,:,1) = eye(4);

    lidar_data.tformRel = zeros(4, 4, lidar_data.odom_range);
    lidar_data.tformRel(:,:,1) = eye(4);

    if (plot_odom)
        figure(k);
        subplot(121); axis equal; title([comment, ': in World']); hold on;
        subplot(122); axis equal; title([comment, ': in Lidar']); hold on;
    end

    for frame = 1:length(odom_msg)
        tform = eye(4,4);
        tform(1:3, 4) = [odom_msg{frame}.Pose.Pose.Position.X, ...
                            odom_msg{frame}.Pose.Pose.Position.Y, odom_msg{frame}.Pose.Pose.Position.Z];
        tform(1:3, 1:3) = quat2rotm([odom_msg{frame}.Pose.Pose.Orientation.W, odom_msg{frame}.Pose.Pose.Orientation.X, ...
                            odom_msg{frame}.Pose.Pose.Orientation.Y, odom_msg{frame}.Pose.Pose.Orientation.Z]);

        lidar_data.tformAbs_world(:, :, frame) = tform;
%         tform

        if frame ~= 1
            lidar_data.tformAbs_lidar(:, :, frame) = lidar_data.tformAbs_world(:, :, 1)^(-1) * lidar_data.tformAbs_world(:, :, frame);
            lidar_data.tformRel(:, :, frame) = lidar_data.tformAbs_lidar(:, :, frame-1)^(-1) * lidar_data.tformAbs_lidar(:, :, frame);

            % add noise to lidar_data.tformRel
            % Batch Continuous-Time Trajectory Estimation section: perturbation
            if (b_noise)
                v_mu = zeros(6, 1);
                v_Sigma = diag([sigma_t, sigma_t, sigma_t, 0, 0, sigma_r]).^2;
                se3_delta_xi = mvnrnd(v_mu, v_Sigma)'; % [rho, phi]
                SE3_delta_tform = SE3.exp(se3_delta_xi);
                tform = lidar_data.tformRel(:, :, frame);
                SE3_tform = SE3_delta_tform * SE3(tform); % tform = delta_tform*tform

                tform = SE3_to_matrix(SE3_tform);
                lidar_data.tformRel(:, :, frame) = tform;
                lidar_data.tformAbs_lidar(:, :, frame) = ...
                    lidar_data.tformAbs_lidar(:, :, frame-1) * lidar_data.tformRel(:, :, frame);

                tform = lidar_data.tformRel(:, :, frame);
                rotValidate(tform(1:3, 1:3));
                tform = lidar_data.tformAbs_lidar(:, :, frame);
                rotValidate(tform(1:3, 1:3));
            end
        end
        if (plot_odom)
            x = reshape(lidar_data.tformAbs_world(1, 4, :), [1,length(odom_msg)]);
            y = reshape(lidar_data.tformAbs_world(2, 4, :), [1,length(odom_msg)]);
            z = reshape(lidar_data.tformAbs_world(3, 4, :), [1,length(odom_msg)]);
            subplot(121); plot3(x, y, z, 'r-'); axis equal; hold on;
            plot3(0, 0, 0, 'b*');
            xlabel('Z (m)'); ylabel('X (m)'); zlabel('Y (m)');

            x = reshape(lidar_data.tformAbs_lidar(1, 4, :), [1,length(odom_msg)]);
            y = reshape(lidar_data.tformAbs_lidar(2, 4, :), [1,length(odom_msg)]);
            z = reshape(lidar_data.tformAbs_lidar(3, 4, :), [1,length(odom_msg)]);
            subplot(122); plot3(x, y, z, 'b-'); axis equal; hold on;
            plot3(0, 0, 0, 'b*');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            drawnow;
        end
    end

end
