Build using script "build.bat", which builds specifically for Visual Studio 15 2017 Win64

build.bat relies on vcpkg, so update the toolchain file location in build.bat

Required vcpkg ports: boost:x64-windows eigen3:x64-windows opencv:x64-windows pangolin:x64-windows tclap:x64-windows
Optional vcpkg ports: kinectsdk2:x64-windows (untested, it also works to install this with the standard windows installer, if you want to use vcpkg you may need to remove "FindKinectSDK2.cmake" from ./Cmake/Modules)

