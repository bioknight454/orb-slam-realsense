cara install

git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3

cd ORB_SLAM3

chmod +x build.sh

./build.sh

ubah isi codingan di example/Stereo/stereo_euroc.cc dengan stereo_euroc.cc

cara running
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
