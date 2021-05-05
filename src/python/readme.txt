rosrun camera_calibration cameracalibrator.py -s 9x6 -q 0.024555 right:=/stereo_camera/right/image_raw left:=/stereo_camera/left/image_raw right_camera:=/stereo_camera/right left_camera:=/stereo_camera/left

cd thesis_ws
catkin_make
source devel/setup.bash
roslaunch camera1394stereo stereo_camera.launch

ROS_NAMESPACE=stereo_camera rosrun stereo_image_proc stereo_image_proc
rosrun rqt_reconfigure rqt_reconfigure
rosrun image_view stereo_view stereo:=stereo_camera image:=image_rect

rosrun data_pkg 1_detection.py
rosrun data_pkg avoidance.py
rosnode kill My_Detection
rostopic hz OBSTT
rosbag play -l -r 20 -q dataset/2107.bag

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/


