# MFfCUAVSLAM #
Map Fusion for Collaborative UAV SLAM

This project was a semester project I did at the Vision for Robotics Lab (www.v4rl.ethz.ch) at the Swiss Federal Institute of Technology (ETH) durgin my master studies in Information Technology and Electrical Engineering.

# Report #
The report that I had to write for this semester project is written in LaTeX and located in the folder report.

# Presentation #
The presentation I gave is located in the folder presentation. There is also an extended version, which does describe certain things in more detail, e.g. the different optimization algorithms.

# Contained scripts/ROS nodes #

##Scripts##

###evaluation/scripts/evaluation.py###
Python script to calculate the root mean squared error (RMSE) from a recorded experiment.

###evaluation/scripts/evaluation_with_offset_estimate.py###
Python script to calculate the root mean squared error (RMSE) from a recorded experiment with an estimated time offset.

##ROS nodes##

###record_vicon###
Receives ground truth positions and writes it into a text file.

###retime_messages###
Republishes topics with retimed timestamps.

###v4rl_mcpslam-mapfusion###
Modifid multi client SLAM system, in which the proposed approaches of this semester project are implemented.

##Evaluation##
The text files with the recorded timestamps and coordinates of each experiment/dataset are in the folder evaluation.

The data sets are "close", "far", "frontal", and "uav". "vicon" represents the ground truth. mX_skY describes how many KeyFrameMatches were required to fuse the maps (X) and how many KeyFrames were skipped (Y) after a KeyFrameMatch was detected.

##Papers##
The relevant research papers and some slides are in the folder papers.
