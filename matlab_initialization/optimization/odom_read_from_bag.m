function lidar_data = odom_read_from_bag(bag_file)

    lidar_data.bag_file = bag_file;
    lidar_data.bag_select = rosbag(lidar_data.bag_file);

    %% read initial extrinsic
    topic_select = select(lidar_data.bag_select, 'Topic', "/T_ext_ini");
    odom_msg = readMessages(topic_select, 'DataFormat', 'struct');
    lidar_data.odom_range = length(odom_msg);
    lidar_data.tformExt = zeros(4, 4, lidar_data.odom_range);
    for frame = 1:length(odom_msg) 
        tform = eye(4,4);
        tform(1:3, 4) = [odom_msg{frame}.Pose.Pose.Position.X, ...
                            odom_msg{frame}.Pose.Pose.Position.Y, odom_msg{frame}.Pose.Pose.Position.Z];
        tform(1:3, 1:3) = quat2rotm([odom_msg{frame}.Pose.Pose.Orientation.W, odom_msg{frame}.Pose.Pose.Orientation.X, ...
                            odom_msg{frame}.Pose.Pose.Orientation.Y, odom_msg{frame}.Pose.Pose.Orientation.Z]); 
        lidar_data.tformExt(:, :, frame) = single(tform);    
    end

    %% read initiali odom
    topic_select = select(lidar_data.bag_select, 'Topic', "/T_odom_ini");
    odom_msg = readMessages(topic_select, 'DataFormat', 'struct');
    lidar_data.odom_range = length(odom_msg);
    lidar_data.tformOdom = zeros(4, 4, lidar_data.odom_range);
    for frame = 1:length(odom_msg) 
        tform = eye(4,4);
        tform(1:3, 4) = [odom_msg{frame}.Pose.Pose.Position.X, ...
                            odom_msg{frame}.Pose.Pose.Position.Y, odom_msg{frame}.Pose.Pose.Position.Z];
        tform(1:3, 1:3) = quat2rotm([odom_msg{frame}.Pose.Pose.Orientation.W, odom_msg{frame}.Pose.Pose.Orientation.X, ...
                            odom_msg{frame}.Pose.Pose.Orientation.Y, odom_msg{frame}.Pose.Pose.Orientation.Z]); 
        lidar_data.tformOdom(:, :, frame) = single(tform);    
    end

    %% read ICP
    topic_select = select(lidar_data.bag_select, 'Topic', "/T_icp");
    odom_msg = readMessages(topic_select, 'DataFormat', 'struct');
    lidar_data.odom_range = length(odom_msg);
    lidar_data.tformICP = zeros(4, 4, lidar_data.odom_range);
    for frame = 1:length(odom_msg) 
        tform = eye(4,4);
        tform(1:3, 4) = [odom_msg{frame}.Pose.Pose.Position.X, ...
                            odom_msg{frame}.Pose.Pose.Position.Y, odom_msg{frame}.Pose.Pose.Position.Z];
        tform(1:3, 1:3) = quat2rotm([odom_msg{frame}.Pose.Pose.Orientation.W, odom_msg{frame}.Pose.Pose.Orientation.X, ...
                            odom_msg{frame}.Pose.Pose.Orientation.Y, odom_msg{frame}.Pose.Pose.Orientation.Z]); 
        lidar_data.tformICP(:, :, frame) = single(tform);    
    end

end