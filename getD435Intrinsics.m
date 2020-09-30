function [intrinsics] = getD435Intrinsics()
%GETD435INTRINSICE Returns the intrinsics object for the D435 realsense.
%Must be set to 640x480, if in doubt, check these against the camera_info
%ros topic

intrinsics.fx = 618.00592;
intrinsics.fy = 617.90845;
intrinsics.ppx = 328.1633;
intrinsics.ppy = 239.6679;

intrinsics.width = 640;
intrinsics.height = 480;

end

