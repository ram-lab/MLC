%%
close all; clc;
bag_file = "/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/evaluation/parameter_1/overlap_0.bag";
% bag_file = "/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/evaluation/overlap_9.bag";
lidar_data = odom_read_from_bag(bag_file);

%% add pose
infoMat = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];

pg = robotics.PoseGraph3D;
fprintf("VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n");

pose = tf_to_vector(lidar_data.tformExt(:,:,1));
fprintf("VERTEX_SE3:QUAT 1 %f %f %f %f %f %f %f\n", pose(1), pose(2), pose(3), ...
    pose(5), pose(6), pose(7), pose(4));
addRelativePose(pg, pose, infoMat);

for i = 1:1:length(lidar_data.tformOdom)
    if i == 1      
        pose = tf_to_vector(lidar_data.tformOdom(:,:,i)); 
    else 
        pose = tf_to_vector(lidar_data.tformOdom(:,:,i-1)^(-1)* ...
                    lidar_data.tformOdom(:,:,i));
    end
    addRelativePose(pg, pose, infoMat);
    fprintf("VERTEX_SE3:QUAT %d %f %f %f %f %f %f %f\n", i+1, pose(1), pose(2), pose(3), ...
        pose(5), pose(6), pose(7), pose(4));    
end

%% add loop closure
for i = 1:1:length(lidar_data.tformICP)
    infoMat = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1]*10000;
    edge = tf_to_vector(lidar_data.tformICP(:,:,i));
    addRelativePose(pg, edge, infoMat, 1, i+2); 
    fprintf("EDGE_SE3:QUAT %d %d %f %f %f %f %f %f %f 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n", ...
        1, i+1, edge(1), edge(2), edge(3), ...
        edge(5), edge(6), edge(7), edge(4));     
end

figure;
hold on;
title('Pose Graph Before Optimization');
xlabel('X [m]');
ylabel('Y [m]'); 
axis equal;
show(pg);
hold off;  

[updatedPG, solutionInfo] = optimizePoseGraph(pg);
figure;
hold on;
title('Pose Graph After Optimization');
xlabel('X [m]');
ylabel('Y [m]');
axis equal;
show(updatedPG);
hold off;

solutionInfo

