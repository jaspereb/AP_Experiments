function cameraParams = getCameraParams()
%GETCAMERAPARAMS Generate a cameraParameters object filled with the D435
%realsense parameters

intrinsics = getD435Intrinsics();
ImageSize = [intrinsics.height, intrinsics.width];
IntrinsicMatrix = [intrinsics.fx, 0, 0; 0, intrinsics.fy, 0; intrinsics.ppx, intrinsics.ppy, 1];

cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix, 'ImageSize', ImageSize); 
end

