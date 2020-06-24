function point_cloud = image2points(ImageLoc,sensor,varargin)
%   Converts images (range images from ToF and Velodyne sensors, stereo pairs
%   from the LocCam and PanCam) from the Katwijk dataset into 3D pointclouds.
%   Note: The included calibration mat files for each sensor should
%   be placed in the same directory as the image2points.mat file.
%
%   [point_cloud] = image2points(rangeImage,sensor) generates a pointcloud
%   for the associated rangeImage. The sensor variable tells the function
%   which sensor generated the provided image. The function uses this to
%   determine which calibration parameters to use and which computation to
%   perform to generate a valid 3D pointcloud.

%   rangeImage = file location (eg. 'C:\images\image.png')
%   sensor = 'ToF' or 'Velodyne' or 'PanCam' or 'LocCam'
%
%   [point_cloud] = image2points(rangeImage,sensor,color) generates the
%   same pointcloud, but adds additional fields for color based on
%   the associated intensity images in the case of the ToF or Velodyne
%   sensor, and the left RGB image in the case of the LocCam and PanCam if
%   color = 1.
%
%   [point_cloud] = image2points(rangeImage,sensor,color,plot) generates
%   the same pointcloud, but also visualizes it using the pointCloud
%   functionality of matlab if plot = 1.

if length(varargin) == 1
    color = varargin{1};
    plotFlag = 0;
elseif length(varargin) == 2
    color = varargin{1};
    plotFlag = varargin{2};
else
    color = 0;
    plotFlag = 0;
end

ImageLoc_prefix = strfind(ImageLoc,'_');
ImageLoc_prefix = ImageLoc(1:ImageLoc_prefix(end));

local_version = version;
if str2double(local_version(1:3)) < 8.5
    error('image2points requires Matlab R2015a or higher and the Computer Vision System Toolbox');
    return
end

switch sensor
    case 'ToF'
        load('.\tof_SR4500_calibration.mat')
        rangeImage = imread(strcat(ImageLoc_prefix,'range.png'));

        if color == 1
            intensityImage = imread(strcat(ImageLoc_prefix,'intensity.png'));

            point_cloud(:,:,4) = double(intensityImage);
        end
        rangeImage(rangeImage >= 16380) = NaN; %Sensor reports invalid measurements as Numbers above 16380 so they are removed.
        point_cloud(:,:,3) = sqrt(((double(rangeImage)/2^14)*9.99308193).^2./(xy(:,:,1).^2+xy(:,:,2).^2+1));
        point_cloud(:,:,1) = xy(:,:,1).*point_cloud(:,:,3);
        point_cloud(:,:,2) = xy(:,:,2).*point_cloud(:,:,3);

        if plotFlag == 1
            scenePC = pointCloud(point_cloud(:,:,1:3));
            if color == 1
                 %Convert 14 bit intensity values to 8 bit values for
                 %visualization. The values are bitshifted by 2 beforehand
                 %to improve contrast, but this can be adjusted as desired.
                gray = uint8(bitshift(point_cloud(:,:,4),-2));
                scenePC.Color = repmat(gray,[1 1 3]);
            end
            scenePC = removeInvalidPoints(scenePC);
            roi_indices = findPointsInROI(scenePC,[-inf, inf; -inf, inf; 0, 10]);
            figure(1);showPointCloud(select(scenePC,roi_indices),'MarkerSize',32);
        end

    case 'Velodyne'
        azimuthImage = imread(strcat(ImageLoc_prefix,'azimuth.png'));
        rangeImage = double(bitshift(imread(strcat(ImageLoc_prefix,'range.png')),0));

        azimuthImage = double(azimuthImage)/100;
        rangeImage = double(rangeImage);

        inc_angles = 15:-2:-15;
        inc = repmat(inc_angles',1,1808);

        if color == 1
            intensityImage = double(imread(strcat(ImageLoc_prefix,'intensity.png')));
            point_cloud(:,:,4) = intensityImage;
        end

        point_cloud(:,:,3) = 0.001*double(rangeImage).*sind(inc);
        point_cloud(:,:,1) = 0.001*double(rangeImage).*cosd(inc).*cosd(azimuthImage);
        point_cloud(:,:,2) = 0.001*double(rangeImage).*cosd(inc).*sind(azimuthImage);

        if plotFlag == 1
            scenePC = pointCloud(point_cloud(:,:,1:3));
            if color == 1
                gray = uint8(point_cloud(:,:,4));
                scenePC.Color = repmat(gray,[1 1 3]);
            end
            scenePC = removeInvalidPoints(scenePC);
            roi_indices = findPointsInROI(scenePC,[-inf, inf; -inf, inf; 0, 100]);
            figure(1);showPointCloud(select(scenePC,roi_indices),'MarkerSize',80);
        end

    case 'PanCam'
        load('.\PanCam_calibration.mat')
        point_cloud = computeStereoPoints(ImageLoc,stereoParams,color,plotFlag);

    case 'LocCam'
        load('.\LocCam_calibration.mat')
        point_cloud = computeStereoPoints(ImageLoc,stereoParams,color,plotFlag);

end

function [point_cloud] = computeStereoPoints(rangeImageLoc,stereoParams,color,plotFlag)
if rangeImageLoc(end-4) == '0'
    left_img = imread(rangeImageLoc);
    right_img = imread(strcat(rangeImageLoc(1:end-5),'1.png'));
elseif rangeImageLoc(end-4) == '1'
    left_img = imread(strcat(rangeImageLoc(1:end-5),'0.png'));
    right_img = imread(rangeImageLoc);
else
    disp('No valid image pair was found at this file location.');
    quit;
end
[J1, J2] = rectifyStereoImages(left_img, right_img, stereoParams);

disparityMap = disparity(rgb2gray(J1), rgb2gray(J2), 'DisparityRange', [0 256]);

point_cloud = reconstructScene(disparityMap, stereoParams)/1000;

if color == 1
    uv_map = pixelLocation(point_cloud,stereoParams);
    point_cloud(:,:,4) = interp2(double(left_img(:,:,1)),uv_map(:,:,1),uv_map(:,:,2));
    point_cloud(:,:,5) = interp2(double(left_img(:,:,2)),uv_map(:,:,1),uv_map(:,:,2));
    point_cloud(:,:,6) = interp2(double(left_img(:,:,3)),uv_map(:,:,1),uv_map(:,:,2));
end

if plotFlag == 1
    scenePC = pointCloud(point_cloud(:,:,1:3));
    if color == 1
        scenePC.Color = uint8(point_cloud(:,:,4:6));
    end
    scenePC = removeInvalidPoints(scenePC);
    roi_indices = findPointsInROI(scenePC,[-inf, inf; -inf, inf; 0, 20]);

    figure(1); imshow(cat(3, J1(:,:,1), J2(:,:,2:3)), 'InitialMagnification', 50);
    figure(2); imshow(disparityMap, [0, 64], 'InitialMagnification', 50);
    figure(3); showPointCloud(select(scenePC,roi_indices));
end


function [uv] = pixelLocation(xyz,camParams)
xy = zeros(size(xyz,1),size(xyz,2),2); uv = zeros(size(xyz,1),size(xyz,2),2);

k1 = camParams.CameraParameters1.RadialDistortion(1); k2 = camParams.CameraParameters1.RadialDistortion(2);
xy(:,:,1) = xyz(:,:,1)./xyz(:,:,3); xy(:,:,2) = xyz(:,:,2)./xyz(:,:,3);
r2 = xy(:,:,1).^2 + xy(:,:,2).^2;

xy(:,:,1) = xy(:,:,1).*(1+k1*r2 + k2*r2.^2);
xy(:,:,2) = xy(:,:,2).*(1+k1*r2 + k2*r2.^2);

uv(:,:,1) = camParams.CameraParameters1.FocalLength(1)*xy(:,:,1) + camParams.CameraParameters1.PrincipalPoint(1);
uv(:,:,2) = camParams.CameraParameters1.FocalLength(2)*xy(:,:,2) + camParams.CameraParameters1.PrincipalPoint(2);
