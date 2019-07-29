function [ lidar_data ] = gen_lidar_tf(pcd_path)

% setup
    lidar_data.path = pcd_path;
    lidar_data.files = dir([pcd_path '*.pcd']);
    lidar_data.files_range = length(lidar_data.files);
    lidar_data.tformRel = zeros(4, 4, lidar_data.files_range);
    lidar_data.tformRel(:,:,1) = eye(4);
    lidar_data.tformAbs = zeros(4, 4, lidar_data.files_range);
    lidar_data.tformAbs(:,:,1) = eye(4);

    ptcloud = pcread([lidar_data.path lidar_data.files(1).name]);

    for frame = 2:lidar_data.files_range
        fprintf('Finding Transform for scan %i\n', frame-1);

        ptcloud_old = ptcloud;
        ptcloud = pcread([lidar_data.path lidar_data.files(i).name]);

        icp_tform = pcregistericp(ptcloud_old, ptcloud,'Metric','pointToPlane',...
                                'Extrapolate',true);
        lidar_data.tformRel(:, :, frame) = icp_tform.T;
        lidar_data.tformAbs(:, :, frame) = lidar_data.tformAbs(:, :, frame-1) * lidar_data.tformRel(:, :, frame);

    end

end
