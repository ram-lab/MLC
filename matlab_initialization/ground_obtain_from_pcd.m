T_top_front = ...
    [0.9982   -0.0436    0.0406   -0.0791; ...
     0.0435    0.9990    0.0037   -0.0206; ...
     -0.0408   -0.0019    0.9992        0; ...
     0         0         0    1.0000];

T_top_tail =  ...
    [-0.9994    0.0310   -0.0123   -1.9033; ...
     -0.0308   -0.9993   -0.0196   -0.0116; ...
     -0.0129   -0.0192    0.9997         0; ...
     0         0         0    1.0000];

T_front_tail = ...
    [-0.9990   -0.0282    0.0354   -2.1709; ...
     0.0272   -0.9992   -0.0277   -0.0986; ...
     0.0362   -0.0267    0.9990   0; ...
     0         0         0    1.0000];
 
ground_top = pcread("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/ground_top.pcd");
ground_front = pcread("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/ground_front.pcd");
ground_tail = pcread("/home/jjiao/catkin_ws/src/localization/LeGO-LOAM/LeGO-LOAM/data/yellow_20190104/8_shape/ground_tail.pcd");

%% plane fitting
maxDistance = 0.05;
maxAngularDistance = 5;
referenceVector = [0,0,1];

[model1,inlierIndices,outlierIndices] = pcfitplane(ground_top,...
            maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ground_top,inlierIndices);
% figure, pcshow(ground_top);
% figure, title('top: ground'), pcshow(plane1);

[model2,inlierIndices,outlierIndices] = pcfitplane(ground_front,...
            maxDistance,referenceVector,maxAngularDistance);
plane2 = select(ground_front,inlierIndices);
% figure, pcshow(ground_front);
figure, title('front: ground'), pcshow(plane2);

[model3,inlierIndices,outlierIndices] = pcfitplane(ground_tail,...
            maxDistance,referenceVector,maxAngularDistance);
plane3 = select(ground_tail,inlierIndices);
% figure, pcshow(ground_tail);
figure, title('tail: ground'), pcshow(plane3);

%% calculate tz
fprintf('Ground Alignment: front-tail:\n');
ground_top = plane1;
ground_front = plane2;
ground_tail = plane3;
p1 = mean(ground_top.Location)';
p2 = mean(ground_front.Location)';
p3 = mean(ground_tail.Location)';

% t_top_front = p1 - T_top_front(1:3, 1:3)*p2;
% t_top_tail = p1 - T_top_tail(1:3, 1:3)*p3;
% t_top_front(3)
% t_top_tail(3)

t_front_tail = p2 - T_front_tail(1:3, 1:3) * p3;
disp("front-tail: tz")
disp(t_front_tail(3));

close all;













