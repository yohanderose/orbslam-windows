# orbslam-windows
### Summary
This repository is based on ORB_SLAM2 and the related publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

The ORB_SLAM2 project contains some code modified from the original version, and contains the additions:
* OS specific changes to allow building on Windows based on Phylliida's fork: https://github.com/Phylliida/orbslam-windows
* Binary ORB vocabulary for faster loading time based on poine's pull request: https://github.com/raulmur/ORB_SLAM2/pull/21 
* Map re-use code from MathewDenny's fork: https://github.com/MathewDenny/ORB_SLAM2
* Initialisation tweaks from users on the issue tracker: https://github.com/raulmur/ORB_SLAM2/issues/59
* Edit mode switch to allow for deleting points from a SLAM map

### Projects
* bin_vocabulary: converts the text-based ORB vocabulary to binary
* map_editor: loads up a map into ORB_SLAM with no camera inputs, purely for editing, and saves the map
* mono_euroc: default example program
* mono_video: example program that runs ORB_SLAM with a webcam or a video file, with an AR window
* mono_webcam_ar: default example AR program
* ORB_SLAM2: modified slam project
* pano_video: example program that runs ORB_SLAM with multiple pre-extracted faces of a panoramic video in a sequence
* rgbd_kinect2: example program that runs ORB_SLAM in RGB-D mode with a live stream from a connected Kinect 2
* stereo_euroc: default example program
* stereo_webcam: example program that runs ORB_SLAM with 2 webcams or 2 folders of timestamped video frames, with an AR window


### Building with vcpkg
Build using script 'build.bat', which builds specifically for Visual Studio 15 2017 Win64 (you can modify this to match your installed generator). The build script relies on vcpkg, so update the toolchain file location in build.bat.

Required vcpkg ports: boost:x64-windows eigen3:x64-windows opencv:x64-windows pangolin:x64-windows tclap:x64-windows

Optional vcpkg ports: kinectsdk2:x64-windows (untested, it also works to install this with the standard windows installer. If you want to use vcpkg to find KinectSDK you may need to remove "FindKinectSDK2.cmake" from ./Cmake/Modules)

