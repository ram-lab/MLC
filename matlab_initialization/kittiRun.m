%% user set variables

%data range
range = 3000:3300;

%path to data
kittiPath = '/Monster/dataset/KITTI/2011_09_30/2011_09_30_drive_0028_sync';
%Sets if camera transform will be plotted
plotCam = false;
%Sets if velodyne transform will be plotted
plotVel = false;

%% add external function directories
addpath('./libicp/matlab');
addpath('./psopt');
addpath('./kabsch');

%% process cameras
camData = genKittiCam(kittiPath, plotCam, range);
camTform = camData.tformRel(:,:,2:end); % 4x4xn

%% process velodyne
velData = genKittiVel(kittiPath, plotVel, range);
velTform = velData.tformRel(:,:,2:end); % 4x4xn

%% find transforms

[Rmean, Rvar] = calcR(velTform, camTform);

[Tmean, Tvar] = calcT(velTform, camTform, Rmean);

%output
[r1,r2,r3] = dcm2angle(Rmean);
out = [180*[r1,r2,r3]/pi, Tmean];
fprintf('angle: %f %f %f translation: %f %f %f\n',out(1),out(2),out(3),out(4),out(5),out(6));

%% new tform

initalGuess = [[Rmean,Tmean'];0,0,0,1];
Trange = [[Rvar,Tvar'];0,0,0,1];

%setup camera
% K = gpuArray(single([camData.K,[0;0;0]]));

%read in and process images
scanIdx = 1:10:size(camData.files,1);
images = cell(size(scanIdx(:),1),1);
for i = 1:length(images(:))
    images{i} = imread([camData.folder, camData.files(scanIdx(i)).name]);
    images{i} = LevImage(images{i});
%     images{i} = gpuArray(single(images{i}));
    if(i == 1)
        avImg = images{i};
    else
        avImg = avImg + images{i};
    end
end

avImg = avImg/i;
for i = 1:length(images(:))
    images{i} = images{i} - avImg;
end

%read in and process lidar
scanIdx = 1:10:size(velData.files,1);
scans = cell(size(scanIdx(:),1),1);
for i = 1:length(scans(:))
    scans{i} = ReadKittiVelDataSingle([velData.folder, velData.files(scanIdx(i)).name]);
    scans{i} = LevLidar(scans{i});
    scans{i} = gpuArray(single(scans{i}));
end

%% optimize
res = OptimizePop(initalGuess, Trange, K, scans, images);



