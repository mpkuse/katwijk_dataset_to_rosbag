%function point_cloud = image2points_velodyne( )
        DB_PATH = "/home/manohar/workspace/bags/external_slam_datasets/katwijk-beach/Part1/Velodyne/"
        ImageLoc_prefix = strcat(DB_PATH, "/Velodyne_2015_11_26_12_53_34_667_")
        plotFlag = 0
        color = 1

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
        
%        if plotFlag == 1
%            scenePC = pointCloud(point_cloud(:,:,1:3));
%            if color == 1
%                gray = uint8(point_cloud(:,:,4));
%                scenePC.Color = repmat(gray,[1 1 3]);
%            end
%            scenePC = removeInvalidPoints(scenePC);
%            roi_indices = findPointsInROI(scenePC,[-inf, inf; -inf, inf; 0, 100]);
%            figure(1);showPointCloud(select(scenePC,roi_indices),'MarkerSize',80);
%        end
