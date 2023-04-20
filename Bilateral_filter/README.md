# Implementation of Bilteral-Filter on Rosario Dataset
## Background
Filtering is an essential technique in image processing and computer vision, where the value of a filtered image at a specific location is determined based on the values of nearby pixels in the input image. Gaussian low-pass filtering is an example of such filtering, which calculates a weighted average of pixel values in a neighborhood, with the weights decreasing as the distance from the center of the neighborhood increases. This approach is suitable for images that change slowly over space since nearby pixels are likely to have similar values. As a result, averaging them preserves the signal while reducing noise. However, this approach is not effective for images with edges as they are blurred by linear low-pass filtering. Many techniques have been developed to avoid this undesired effect, and bilateral filtering is a simple, non-iterative method that preserves edges while smoothing smooth regions.

## Run ROS bagfiles of Rosario Dataset through ORB_SLAM2 
After install ORB_SALM2 and ROS, call roscore
```
roscore
```
Replace the ORBextractor.cc from the orignial ORB_SLAM2 with the [ORBextractor.cc](https://github.com/yuxinton/Bilteral-Filter/blob/main/ORBextractor.cc) in this repo.

Recompile ORB_SLAM2 and add the directory to the 'ros_package_path', and then build the project.
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/yuxinton/ORB_SLAM2/Examples/ROS
./build.sh
./build_ros.sh
```
Download the orbslam_ros.yaml file from the web and put it in ORB_SALM2，synchronize ROS clock with the simulator and run the ORB_SLAM2 stereo camera node with the specified vocabulary and configuration files.
```
rosparam set use_sim_time true 
rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/orbslam_ros.yaml.txt true 
```
Extract the sequence bag files and feed into ORB_SLAM2 with 0.5 speed.
```
rosparam set use_sim_time true 
rosbag play sequence03.bag --clock -r 0.5 
```
After playing the bag file, exit ORB_SLAM2 and save the trajectory.
## Evaluation using [evo](https://github.com/MichaelGrupp/evo)
Install the evaluation software [evo](https://github.com/MichaelGrupp/evo)
```
sudo apt install python-pip 
pip install evo --upgrade --no-binary evo 
```
After installing the evaluation software, evaluate the absolute pose error.
```
evo_ape tum truth.txt before_filter.txt -p -va
evo_ape tum truth.txt after_filter.txt -p -va 
```
Generate the comparison for trajectory by 
```
evo_traj tum --ref=truth.txt before_filter.txt after_filter.txt -p -va 
```
To compare the ASE(Absolute Pose Errors) of results before and after implementing bilateral filter. Make a new directory, evaluate the ASE of both results and save them as zip files in that directory 
```
mkdir results
evo_ape tum truth.txt before.txt -va --plot --plot_mode xyz --save_results results/before.zip
evo_ape tum truth.txt after.txt -va --plot --plot_mode xyz --save_results results/after.zip
```
Generate the ASE comparison plots by 
```
evo_res results/*.zip -p --save_table results/table.csv
```
## Calculate Inference Time
Rebuild ORB_SLAM2 with the modified [ros_stereo.cc](https://github.com/dansim-umich/ORB_SLAM2_Bilateral_LoFTR/blob/master/Bilateral_filter/ros_stereo.cc).
The modified  [ros_stereo.cc](https://github.com/dansim-umich/ORB_SLAM2_Bilateral_LoFTR/blob/master/Bilateral_filter/ros_stereo.cc) utilized chrono library to calculate inference time.
```
./build.sh
./build_ros.sh
```
Run the rebuilt ORB_SLAM2
```
rosparam set use_sim_time true 
rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/orbslam_ros.yaml.txt true 
```
Play the sequence bag file and the inference time will be printed to command window.
```
rosparam set use_sim_time true 
rosbag play sequence03.bag --clock -r 0.5 
```


