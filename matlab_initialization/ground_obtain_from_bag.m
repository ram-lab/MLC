function [ ground_centroid ] = ground_obtain_from_bag(bag_file, topic)

    bag_select = rosbag(bag_file);
    topic_select = select(bag_select, 'Topic', topic);

    for i = 1:topic_select.NumMessages
        cloud_ground = readMessages(topic_select, i, 'DataFormat', 'PointCloud2');
        xyz = readXYZ(cloud_ground{1});
        c = mean(xyz);
        ground_centroid{i} = c;
    end
end
