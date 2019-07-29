ref = pcread('/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/yellow_20180104/top_front/raw/ref_filter.pcd');
data = pcread('/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/yellow_20180104/top_front/raw/data_filter.pcd');
% pcshow(ref, [228/255 26/255 28/255]); hold on;
% pcshow(data, [9/255 59/255 170/255]);

% [228/255 26/255 28/255; 9/255 59/255 170/255]

% pcshowpair(ref, data, 'MarkerSize', 50);



T_gt_12 = [ ...
      0.99612    -0.033733     0.081251      0.42039;...
     0.034646      0.99935   -0.0098511  -0.00066243;...
    -0.080866     0.012628      0.99664      -1.2597;...
            0            0            0            1];    
T_gt_13 = [ ...
     -0.99954     0.029889   -0.0044847      -2.1;...
    -0.029799     -0.99938    -0.018984     0.063455;...
   -0.0050493    -0.018841      0.99981      -1.1817;...
            0            0            0            1];   
        
T_12 = [           
      0.99592    -0.042121     0.079841      0.43473;
     0.043067      0.99902    -0.010168   -0.0021749;
    -0.079335     0.013565      0.99676      -1.2604;
            0            0            0            1];
T_13 = [
     -0.99985    0.0034442    -0.017156      -2.1711;
   -0.0030873     -0.99978    -0.020786     0.087945;
    -0.017224    -0.020729      0.99964      -1.1794;
            0            0            0            1];
        
%%
tform = affine3d();
tform.T(1:3,1:3) = T_gt_12(1:3,1:3);
tform.T(4, 1:3) = T_gt_12(1:3,4)';
data_trans = pctransform(data, tform);
figure;
pcshowpair(ref, data_trans, 'MarkerSize', 50);
view(0,90) 

%%
tform = affine3d();
tform.T(1:3,1:3) = T_12(1:3,1:3);
tform.T(4, 1:3) = T_12(1:3,4)';
data_trans = pctransform(data, tform);
figure;
pcshowpair(ref, data_trans, 'MarkerSize', 50);
view(0,90) 




