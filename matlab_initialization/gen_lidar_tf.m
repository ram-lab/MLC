function [ lidar_data ] = gen_lidar_tf(pcd_path, plot_lidar, lidar_type)

    % setup
    lidar_data.path = pcd_path;
    lidar_data.files = dir([pcd_path '*.pcd']);
    lidar_data.files_range = length(lidar_data.files);
    lidar_data.tformRel = zeros(4, 4, lidar_data.files_range);
    lidar_data.tformRel(:,:,1) = eye(4);
    lidar_data.tformAbs = zeros(4, 4, lidar_data.files_range);
    lidar_data.tformAbs(:,:,1) = eye(4);    
    
    if (plot_lidar)
        figure;
        axis equal;
        title(['Transformation of ', lidar_type, ' Lidar']);
        hold on;
    end    
    
    ptcloud = pcread([lidar_data.path lidar_data.files(1).name]);
    
    x = []; y = []; z = [];
%     for frame = 2:lidar_data.files_range
    for frame = 2:200
        fprintf('Finding Transform for scan %i\n', frame-1);
        
        ptcloud_old = ptcloud;
        ptcloud = pcread([lidar_data.path lidar_data.files(frame).name]);
        
        icp_tform = pcregistericp(ptcloud_old, ptcloud,'Metric','pointToPlane',...
                                'Tolerance', [0.005, 0.1]);
        tform = icp_tform.T;
        tform = [[tform(1:3,1:3),tform(4,1:3)']; 0 0 0 1];
        
        lidar_data.tformRel(:, :, frame) = tform;
        lidar_data.tformAbs(:, :, frame) = lidar_data.tformAbs(:, :, frame-1) * lidar_data.tformRel(:, :, frame);
%         lidar_data.tformAbs(:, :, frame)
        
        x = [x;lidar_data.tformAbs(1,4,frame)];
        y = [y;lidar_data.tformAbs(2,4,frame)];
        z = [z;lidar_data.tformAbs(3,4,frame)];
        if (plot_lidar)
            plot(x, y, 'r-');
            drawnow;
        end
    end
    
%     if (plot_lidar)
%         plot3(x, y, z, 'r-');
%         title(['Transformation of ', lidar_type, ' Lidar']);
%         drawnow;
%     end
end