#include <librealsense2/rs.hpp>  // RealSense SDK
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "System.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_realsense path_to_vocabulary path_to_settings path_to_times_file (trajectory_file_name)" << endl;
        return 1;
    }

    // Initialize RealSense pipeline
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);  // RGB Stream
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);  // Depth Stream
    p.start(cfg);

    // SLAM system setup
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    vector<float> vTimesTrack;
    vTimesTrack.resize(5000);  // Size of tracking times array

    cv::Mat imLeft, imRight, depthImg;

    while (true)
    {
        rs2::frameset frames = p.wait_for_frames();  // Wait for the next set of frames from the camera
        rs2::frame color_frame = frames.get_color_frame();  // Get the color frame
        rs2::frame depth_frame = frames.get_depth_frame();  // Get the depth frame

        // Convert RealSense frames to OpenCV Mats
        imLeft = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        imRight = imLeft.clone();  // For simplicity, use the same image for both left and right (for stereo)

        double tframe = chrono::duration_cast<chrono::duration<double>>(chrono::system_clock::now().time_since_epoch()).count();

        // Process the stereo frames with the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(), "RealSense_Frame");

        // Optional: Display the frames
        cv::imshow("Left Image", imLeft);
        cv::imshow("Right Image", imRight);
        if (cv::waitKey(1) == 'q') break;  // Exit loop if 'q' is pressed
    }

    // Shutdown the SLAM system and save trajectories
    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    // If using RealSense, this function is no longer needed
    // This was only used for loading images from files.
}
