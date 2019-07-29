read_from_mat = true;
str_ini_X_mat = 'data/8_shape_ini_X.mat';
str_ground_centroid_mat = 'data/8_shape_ground_centroid.mat';

load(str_ini_X_mat);
if read_from_mat
    load(str_ground_centroid_mat);
    lidar_1_data.ground_centroid = ground_centroid_1;
    lidar_2_data.ground_centroid = ground_centroid_2;
    lidar_3_data.ground_centroid = ground_centroid_3;
else
    bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_top.bag';
    lidar_1_data.ground_centroid = ground_obtain_from_bag(bag_file, '/ground_cloud');
    bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_front.bag';
    lidar_2_data.ground_centroid = ground_obtain_from_bag(bag_file, '/ground_cloud');
    bag_file = '/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/odom_tail.bag';
    lidar_3_data.ground_centroid = ground_obtain_from_bag(bag_file, '/ground_cloud');
end

R = ini_X(1:3,1:3);
v_t = [];
gc_1 = datasample(lidar_1_data.ground_centroid, 50);
gc_3 = datasample(lidar_3_data.ground_centroid, 50);
for i = 1:length(gc_1)
    p1 = gc_1{i}';
    p3 = gc_3{i}';
    t = p1 - R*p3;
    v_t = [v_t, p1 - R*p3];
end

v_mean_t = mean(v_t, 2);
v_std_t = std(v_t, 0, 2);
disp('t_z mean:'); disp(v_mean_t(3));
disp('t_z std:'); disp(v_std_t(3));




