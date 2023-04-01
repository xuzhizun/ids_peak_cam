# ids_peak_cam
ROS driver for ids cam U3 with the peak SDK

It is a simple node, in which only the frameRate and exposureTime can be adjusted.


Prerequisition

Install ids peak driver.


Start

In launch files, replace the serial number with that of your own device, and set the exposureTime and frameRate for the camera.

Runing 

roslaunch ids_peak_cam ids_cam.launch 

Published topics:

/ids_peak_cam/camera_info
/ids_peak_cam/image_raw
/ids_peak_cam/image_raw/compressed
/ids_peak_cam/image_raw/compressed/parameter_descriptions
/ids_peak_cam/image_raw/compressed/parameter_updates
/ids_peak_cam/image_raw/compressedDepth
/ids_peak_cam/image_raw/compressedDepth/parameter_descriptions
/ids_peak_cam/image_raw/compressedDepth/parameter_updates
/ids_peak_cam/image_raw/theora
/ids_peak_cam/image_raw/theora/parameter_descriptions
/ids_peak_cam/image_raw/theora/parameter_updates


Published services:

/ids_cam_publisher/get_loggers
/ids_cam_publisher/set_logger_level
/ids_peak_cam/image_raw/compressed/set_parameters
/ids_peak_cam/image_raw/compressedDepth/set_parameters
/ids_peak_cam/image_raw/theora/set_parameters
/ids_peak_cam/set_camera_info


