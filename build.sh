echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir build
cd build
cmake -G "Visual Studio 14 2015 Win64" ..
cmake --build . --config Release

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir build
cd build
cmake -G "Visual Studio 14 2015 Win64" ..
cmake --build . --config Release

echo "Uncompress vocabulary ..."
cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake -G "Visual Studio 14 2015 Win64" ..
cmake --build . --config Release
