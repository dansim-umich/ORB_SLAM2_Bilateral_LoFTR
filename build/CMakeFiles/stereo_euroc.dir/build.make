# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dansim/ORB_SLAM2_Bilateral_LoFTR/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_euroc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_euroc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_euroc.dir/flags.make

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o: CMakeFiles/stereo_euroc.dir/flags.make
CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o: /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dansim/ORB_SLAM2_Bilateral_LoFTR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o -c /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc.cc

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc.cc > CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.i

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc.cc -o CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.s

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.requires:

.PHONY : CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.requires

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.provides: CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.requires
	$(MAKE) -f CMakeFiles/stereo_euroc.dir/build.make CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.provides.build
.PHONY : CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.provides

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.provides.build: CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o


# Object files for target stereo_euroc
stereo_euroc_OBJECTS = \
"CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o"

# External object files for target stereo_euroc
stereo_euroc_EXTERNAL_OBJECTS =

/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: CMakeFiles/stereo_euroc.dir/build.make
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/lib/libORB_SLAM2.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_shape.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_stitching.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_objdetect.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_superres.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_videostab.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_calib3d.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_features2d.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_flann.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_highgui.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_ml.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_photo.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_video.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_videoio.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_imgproc.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_viz.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/local/lib/libopencv_core.so.3.2.0
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /home/dansim/Pangolin/build/src/libpangolin.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libGL.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libGLEW.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libSM.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libICE.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libX11.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libXext.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libdc1394.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libavcodec.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libavformat.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libavutil.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libswscale.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/libOpenNI.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/libOpenNI2.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libpng.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libz.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libjpeg.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libtiff.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libIlmImf.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
/home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc: CMakeFiles/stereo_euroc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dansim/ORB_SLAM2_Bilateral_LoFTR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_euroc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_euroc.dir/build: /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2/Examples/Stereo/stereo_euroc

.PHONY : CMakeFiles/stereo_euroc.dir/build

CMakeFiles/stereo_euroc.dir/requires: CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o.requires

.PHONY : CMakeFiles/stereo_euroc.dir/requires

CMakeFiles/stereo_euroc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_euroc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_euroc.dir/clean

CMakeFiles/stereo_euroc.dir/depend:
	cd /home/dansim/ORB_SLAM2_Bilateral_LoFTR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2 /home/dansim/ORB_SLAM2_Bilateral_LoFTR/ORB_SLAM2 /home/dansim/ORB_SLAM2_Bilateral_LoFTR/build /home/dansim/ORB_SLAM2_Bilateral_LoFTR/build /home/dansim/ORB_SLAM2_Bilateral_LoFTR/build/CMakeFiles/stereo_euroc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_euroc.dir/depend

