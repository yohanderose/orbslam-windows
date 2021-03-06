cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

if(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release)
endif()

# set the Cmake module directory
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/Cmake/Modules)

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "My multi config types" FORCE)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /W0 /MP /bigobj /Z7 /O2")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /INCREMENTAL /DEBUG:FASTLINK")

set(CompilerFlags
	CMAKE_CXX_FLAGS
	CMAKE_CXX_FLAGS_DEBUG
	CMAKE_CXX_FLAGS_RELEASE
	CMAKE_C_FLAGS
	CMAKE_C_FLAGS_DEBUG
	CMAKE_C_FLAGS_RELEASE
)
foreach(CompilerFlag ${CompilerFlags})
	string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
endforeach()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
SET(DBUILD_STATIC true)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCOMPILEDWITHC11)
	message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	add_definitions(-DCOMPILEDWITHC0X)
	message(STATUS "Using flag -std=c++0x.")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 2.4.3 QUIET)
	if(NOT OpenCV_FOUND)
		message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
	endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED
	COMPONENTS serialization filesystem
)

include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIR}
	${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} STATIC
	src/System.cc
	src/Tracking.cc
	src/LocalMapping.cc
	src/LoopClosing.cc
	src/ORBextractor.cc
	src/ORBmatcher.cc
	src/FrameDrawer.cc
	src/Converter.cc
	src/MapPoint.cc
	src/KeyFrame.cc
	src/Map.cc
	src/MapDrawer.cc
	src/Optimizer.cc
	src/PnPsolver.cc
	src/Frame.cc
	src/KeyFrameDatabase.cc
	src/Sim3Solver.cc
	src/Initializer.cc
	src/Viewer.cc
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${Boost_LIBRARIES}
	${Pangolin_LIBRARIES}
	${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/Release/DBoW2.lib
	${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/Release/g2o.lib
)

# Build mono video/webcam project
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono_video
	Examples/Monocular/mono_video.cc
	Examples/Utils/MyARViewer.cc
)
target_link_libraries(mono_video ${PROJECT_NAME})

# Build tracking test project
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(tracking_test
	Examples/Monocular/tracking_test.cc
	Examples/Utils/MyTrackingVisualizer.cc
)
target_link_libraries(tracking_test ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono_image_loc
	Examples/Monocular/mono_image_loc.cc
	Examples/Utils/MyARViewer.cc
)
target_link_libraries(mono_image_loc ${PROJECT_NAME})

# Build kinect2 project
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
find_package(KinectSDK2 QUIET)
if(KinectSDK2_FOUND)
	add_executable(rgbd_kinect2
		Examples/RGB-D/rgbd_kinect2.cc
		Examples/Utils/KinectRgbd.cc
		Examples/Utils/MyARViewer.cc
	)
	target_include_directories(rgbd_kinect2 PRIVATE ${KinectSDK2_INCLUDE_DIRS} )
	target_link_libraries(rgbd_kinect2 
		${PROJECT_NAME}
		${KinectSDK2_LIBRARIES}
	)
endif()

# Build pano project
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Panoramic)
add_executable(pano_video
	Examples/Panoramic/pano_video.cc
	Examples/Utils/MyARViewer.cc
)
target_link_libraries(pano_video ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Utils)
add_executable(map_editor
	Examples/Utils/map_editor.cc
)
target_link_libraries(map_editor ${PROJECT_NAME})

# Build example AR demo
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono_webcam_ar
	Examples/Monocular/mono_webcam_ar.cc
	Examples/Utils/ViewerAR.cc
)
target_link_libraries(mono_webcam_ar ${PROJECT_NAME})

# Build example mono_euroc
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

# Build example stereo_euroc
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

# Build example stereo_webcams
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
add_executable(stereo_webcam
Examples/Stereo/stereo_webcam.cc
Examples/Utils/MyARViewer.cc
Examples/Utils/StereoRectification.cc
Examples/Utils/StereoSync.cc)
target_link_libraries(stereo_webcam 
	${PROJECT_NAME}
	${Boost_LIBRARIES}
)

# Build Tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Utils)
add_executable(bin_vocabulary
	Examples/Utils/bin_vocabulary.cc
)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})

# add_executable(mono_webcam
# Examples/Monocular/mono_webcam.cc)
# target_link_libraries(mono_webcam ${PROJECT_NAME})
# Build examples
# add_executable(mono_tum
# Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})
# add_executable(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
# add_executable(rgbd_tum
# Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum ${PROJECT_NAME})
# add_executable(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})
