echo "Exporting Rosario $1 images"

cd Rosario/Sequence_$1
gnome-terminal --tab --title="roscore" -- roscore
gnome-terminal --tab --title="play" -- rosbag play --clock --pause sequence0$1.bag
mkdir images_left
mkdir images_right
cd images_left
gnome-terminal --tab --title="left" -- rosrun image_view extract_images _sec_per_frame:=0.01 image:=/stereo/left/image_raw
cd ../images_right
gnome-terminal --tab --title="right" -- rosrun image_view extract_images _sec_per_frame:=0.01 image:=/stereo/right/image_raw