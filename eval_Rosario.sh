echo "Evaluating Rosario $1 on ORB_SLAM2"

cd Rosario/Sequence_$1
evo_ape tum sequence0$1_gt.txt FrameTrajectory_TUM_Format_base.txt -p --verbose --align