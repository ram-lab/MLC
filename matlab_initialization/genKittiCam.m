function [ camData ] = genKittiCam( kittiPath, plotCam, range )
%GENLADYBUG Generates SFM points using the four cameras on the kitti car

%setup
camData.folder = [kittiPath '/image_00/data/'];

camData.K = [721.5377 0 609.5593; 0 721.5377 172.8540; 0 0 1];

camData.point = cell(1,1);
    
%get range of data
camData.files = dir([camData.folder,'*.png']);
camData.files = camData.files(range);

%preallocate memory
camData.tformRel = zeros(4,4,size(camData.files(:),1));
camData.tformRel(:,:,1) = eye(4);
camData.tformAbs = zeros(4,4,size(camData.files(:),1));
camData.tformAbs(:,:,1) = eye(4);

%not sure how to preallocate this might be inefficient
C2 = [];
pointIdx = [];


if(plotCam)
    figure;
    axis equal;
    hold on;
end

%setup loop
im = imread([camData.folder camData.files(1).name]); 
if(size(im,3) == 3)
    im = rgb2gray(im);
end

%find transform for each image
for frame = 2:size(camData.files,1)

    fprintf('Finding Transform for image %i of camera %i\n', frame-1, i);

    %store old data
    imOld = im;

    %find sensor transforms

    %read new data
    im = imread([camData.folder camData.files(frame).name]); 
    if(size(im,3) == 3)
        im = rgb2gray(im);
    end

    [camData.tformRel(:,:,frame), C1, C1BaseValid, C2] = ...
    getTcam(imOld, im, ones(size(im)), camData.K,C2);

    %update point index
    pointIdx = pointIdx(C1BaseValid);

    for j = 1:size(pointIdx,1)
        camData.point{pointIdx(j),1}.image = [camData.point{pointIdx(j)}.image;frame];
        camData.point{pointIdx(j),1}.X0 = [camData.point{pointIdx(j),1}.X0;C2(j,1)];
        camData.point{pointIdx(j),1}.Y0 = [camData.point{pointIdx(j),1}.Y0;C2(j,2)];
    end

    j = size(pointIdx,1);
    pointIdx = [pointIdx; (size(camData.point,1)+1:(size(camData.point,1)+size(C1,1)-size(C1BaseValid,1)))'];

    for j = j+1:size(pointIdx,1)
        camData.point{pointIdx(j),1}.image = [frame-1;frame];
        camData.point{pointIdx(j),1}.X0 = [C1(j,1);C2(j,1)];
        camData.point{pointIdx(j),1}.Y0 = [C1(j,2);C2(j,2)];
    end

    %generate absolute camera transformations
    camData.tformAbs(:,:,frame) = camData.tformAbs(:,:,frame-1)*camData.tformRel(:,:,frame);

    if(plotCam)
        %plot points
        plot3(camData.tformAbs(1,4,frame),camData.tformAbs(2,4,frame),camData.tformAbs(3,4,frame));
        drawnow;
    end
end 

camData.point = cell2mat(camData.point(2:end));
end

