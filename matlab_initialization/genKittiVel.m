function [ velData ] = genKittiVel( kittiPath, plotVel, range )
%GENLADYBUG Generates transforms using icp and the velodyne

%setup
velData.folder = [kittiPath '/velodyne_points/data/'];

velData.files = dir([velData.folder,'*.bin']);
velData.files = velData.files(range);
velData.tformRel = zeros(4,4,size(velData.files(:),1));
velData.tformRel(:,:,1) = eye(4);
velData.tformAbs = zeros(4,4,size(velData.files(:),1));
velData.tformAbs(:,:,1) = eye(4);
    
if(plotVel)
    figure;
    axis equal;
    hold on;
end

%setup loop
vel = ReadKittiVelDataSingle( [velData.folder velData.files(1).name] );

%find transform for each velodyne scan
for frame = 2:size(velData.files,1)
    
    fprintf('Finding Transform for scan %i\n', frame-1);

    %store old data
    velOld = vel;

    %read new data
    vel = ReadKittiVelDataSingle([velData.folder velData.files(frame).name]);
    
    %find sensor transforms
    velData.tformRel(:,:,frame) = getTvel(velOld, vel);
    
    %generate absolute transformations
    velData.tformAbs(:,:,frame) = velData.tformAbs(:,:,frame-1)*velData.tformRel(:,:,frame);

    if(plotVel)
        %plot points
        plot3(velData.tformAbs(1,4,frame),velData.tformAbs(2,4,frame),velData.tformAbs(3,4,frame));
        drawnow;
    end
end 

