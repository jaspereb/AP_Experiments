function cameraParams = getCameraParams()
%GETCAMERAPARAMS Generate a cameraParameters object filled with the D435
%realsense parameters. Some of these are redefined for simplicity.

intrinsics = getD435Intrinsics();

intrinsics.fx = 620;
intrinsics.fy = 620;
intrinsics.ppx = 320;
intrinsics.ppy = 240;

ImageSize = [intrinsics.height, intrinsics.width];
IntrinsicMatrix = [intrinsics.fx, 0, 0; 0, intrinsics.fy, 0; intrinsics.ppx, intrinsics.ppy, 1];

cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix, 'ImageSize', ImageSize); 
end

