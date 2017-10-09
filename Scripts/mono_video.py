from util import *
from string import Template
import os

# Set command args
#args = "-W 1280 -H 720 -c 0 -v $PHD_FILES\orbslam\ORBvoc.bin -s $PHD_FILES\orbslam\webcam-senz3d.yaml"
args = "-W 960 -H 540 -f $PHD_FILES\orbslam\s6-01.avi -v $PHD_FILES\orbslam\ORBvoc.bin -s $PHD_FILES\orbslam\mobile-s6-2.yaml"
#args = "-W 960 -H 540 -c 1 -v $PHD_FILES\orbslam\ORBvoc.bin -s $PHD_FILES\orbslam\webcam-f200.yaml"

# Substitute environment variables
t = Template(args)
args = t.substitute(os.environ)

# Run program
runCommand("../Examples/Monocular/Release/mono_video.exe " + args, False)
