# Implementing Bilateral Filter & LoFTR in ORB-SLAM2

This code was made using the [ORB SLAM2](https://github.com/raulmur/ORB_SLAM2) and [LoFTR](https://github.com/zju3dv/LoFTR) Repositories.

This was a final project for ROB 530 at the University of Michigan

Authors:
Anirudh Aatresh, Haoyuan Ma, Ziqing Qin, Daniel Simmons, Yuxin Tong

Submission date: 4/19/2023

# 1. Background
This project attempts to modify **ORB-SLAM2** via two methods: **bilateral filter** and **LoFTR** embedding.

In recent years, there has been a surge in investment from companies towards perception for agricultural robots for various tasks like precision agriculture, weed identification, and crop monitoring. Simultaneous Localization and Mapping (SLAM) has emerged as a popular approach for accurate environment mapping and localization, and visual perception provides cost-efficient and rich feature representations compared to LIDAR. However, developing visual SLAM for agriculture poses challenges such as monotonous rural landscapes, varying outdoor illumination, and camera motions due to irregular terrain. This paper proposes an improved visual SLAM method for farm mapping, addressing the identified challenges and evaluating its performance on the Rosario dataset. The proposed method improves the accuracy and robustness of visual SLAM for agricultural applications.

# 2. Prerequisites
This library is most stable within an **Ubuntu 18.04** environment.
When creating a new linux environment it is necessary to execute:
```
sudo apt-get update
```

# Bilateral Filter
A Bilateral Filter reduces noise in an image by reducing information entropy. A detailed readme could be found under Bilateral_fiter directory 

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## OpenCV 3.2.0
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload with:
```
mkdir OpenCV
cd OpenCV
sudo apt install wget
wget -O https://github.com/opencv/opencv/archive/3.2.0.zip
```
Copy and Paste
```
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
#define AVFMT_RAWPICTURE 0x0020
```
To the top of
```
opencv/modules/videoio/src/cap_ffmpeg_impl.hpp
```
and change:
```
char* str = PyString_AsString(obj);
```
to:
```
const char* str = PyString_AsString(obj);
```
in line 730 of
```
opencv/modules/python/src2/cv2
```

Install instructions can be found at: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html under Detailed Process.
**do not download opencv-4.x**
**When doing OpenCV cmake step add -DENABLE_PRECOMPILED_HEADERS=OFF**

## Eigen 3.2.10
Required by g2o (see below). Download with:
```
wget https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.zip
```
Follow OpenCV install instructions

## Pangolin 0.5
We use Pangolin v0.5 for visualization and user interface. Dowload and install with:
```
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.5
sudo apt-get install libgl1-mesa-dev
sudo apt install libglew-dev
```
Then follow steps install steps in the included ReadMe.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS Melodic
Follow [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) installation. All steps up to 1.6.1

## LoFTR
Install python3 dev tools, create the LoFTR environment, and download the trained weights.
```
cd LoFTR
pip3 install python-dev
conda env create -f environment.yaml
conda activate loftr
pip install torch einops yacs kornia
mkdir weights
cd weights
sudo apt install gdown
gdown 1M-VD35-qdB5Iw-AtbDBCKC7hPolFW9UY
```

# 3. Installation

Clone the repository:
```
git clone https://github.com/dansim-umich/ORB_SLAM2_Bilateral_LoFTR.git
```

## build ORB-SLAM2
We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Execute:
```
cd ORB_SLAM2_Bilateral_LoFTR
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

## build ros
Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open *.bashrc* file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2_Bilateral_LoFTR:

```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2_Bilateral_LoFTR/Examples/ROS
```
Execute:
```
chmod +x build_ros.sh
./build_ros.sh
```
**If Python is found instead of python3:**
in
```
ORB_SLAM2_Bilateral_LoFTR/Examples/ROS/ORB_SLAM2/build/CMakeCache.txt
```
change

```
PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python
```
to

```
PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3
```
and execute:
```
./build_ros.sh
```

# 4. datset
This project was performed with the [Rosario](https://www.cifasis-conicet.gov.ar/robot/doku.php) agricultural dataset. 
For x = 1,2,3,4,5, or 6
```
cd Rosario
mkdir Sequence_x
cd Sequence_x
```
Recommended download, concatenation, and decompression commands are included on the Rosario page.

**For ORB_SLAM2 only the sequence0x.bag and sequence0x_gt.txt files are needed**
**Even single .bag sequences need to be decompressed**

The calibration files are not the appropriate format for *ROS*. The necessary camera calibation is included:
```
ORB_SLAM2_Bilateral_LoFTR/Rosario/orbslam_ros.yaml
```

# 5. Running
The script *run_Rosario.sh* is included to run upon the Rosario dataset. To run sequence x execute:
```
sudo apt install gnome-terminal
chmod +x run_Rosario.sh
./run_Rosario.sh x
```
This will open three terminals: *roscore*, *Sequence*, and *ORB-SLAM2*.
Once the *ORB-SLAM2: Map Viewer* and *ORB-SLAM2: Current Frame* have appeared go to the *Sequence* terminal and hit spacebar to play the rosbag.

Upon completion the *Sequence* terminal will close.
At this point go to the *ORB-SLAM2* terminal and close with *ctrl + c*. This will save the Trajectories in the respective Sequence folder.
The *rosbag* terminal can now also be closed with *ctrl + c*.

# Citations

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

     @article{pire2019rosario,
        author = {Taih{\'u} Pire and Mart{\'i}n Mujica and Javier Civera and Ernesto Kofman},
        title = {The Rosario dataset: Multisensor data for localization and mapping in agricultural environments},
        journal = {The International Journal of Robotics Research},
        volume = {38},
        number = {6},
        pages = {633-641},
        year = {2019},
        doi = {10.1177/0278364919841437}
    }

    @article{sun2021loftr,
        title={{LoFTR}: Detector-Free Local Feature Matching with Transformers},
        author={Sun, Jiaming and Shen, Zehong and Wang, Yuang and Bao, Hujun and Zhou, Xiaowei},
        journal={{CVPR}},
        year={2021}
    }