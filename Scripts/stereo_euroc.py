from util import *
from string import Template
import os

# Set command args
args = "$PHD_FILES\\orbslam\\ORBvoc.bin $PHD_FILES\\orbslam\\examples\\EuRoC-stereo.yaml $PHD_FILES\\orbslam\\examples\\V1_02_medium\\mav0\\cam0\\data $PHD_FILES\\orbslam\\examples\\V1_02_medium\\mav0\\cam1\\data $PHD_FILES\\orbslam\\examples\\EuRoC_TimeStamps\\V102.txt"

# Substitute environment variables
t = Template(args)
args = t.substitute(os.environ)

# Run program
runCommand("../Examples/Stereo/Release/stereo_euroc.exe " + args, False)
