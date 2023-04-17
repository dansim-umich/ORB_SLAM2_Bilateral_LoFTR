echo "Running Rosario $1 on ORB_SLAM2"

cd Rosario/Sequence_$1
gnome-terminal --tab --title="roscore" -- roscore
gnome-terminal --tab --title="Calibrate" -- rosrun ORB_SLAM2 Stereo ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Rosario/orbslam_ros.yaml true
gnome-terminal --tab --title="play" -- rosbag play --clock --pause sequence0$1.bag /stereo/left/image_raw:=/camera/left/image_raw /stereo/right/image_raw:=/camera/right/image_raw