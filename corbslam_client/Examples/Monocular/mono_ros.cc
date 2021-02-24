/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

// class ImageGrabber
// {
// public:
//     ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

//     void GrabImage(const sensor_msgs::ImageConstPtr& msg);

//     ORB_SLAM2::System* mpSLAM;
// };


void imageCallback(const sensor_msgs::ImageConstPtr& msg);

// ORB_SLAM2::System SLAM("/home/nicolas/catkin_corbslam/src/CORB-SLAM/corbslam_client/Vocabulary/ORBvoc.txt", 
//                        "/home/nicolas/catkin_corbslam/src/CORB-SLAM/corbslam_client/Examples/Monocular/KITTI00-02.yaml",
//                         ORB_SLAM2::System::MONOCULAR, true);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_ros");
    // ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun corbslam_client mono_ros <PATH_TO_VOC.txt> <PATH_TO_CONFIG.yaml>" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, imageCallback);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);


    // ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}

// void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
// {
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
// }











/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


// using namespace std;

// bool LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "mono_kitti");


//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

//     // Main loop
//     cv::Mat im;
//     for(int ni=0; ni<nImages; ni++)
//     {
//         // Read image from file
//         im = cv::imread(vstrImageFilenames[ni], IMREAD_UNCHANGED);
//         double tframe = vTimestamps[ni];

//         if(im.empty())
//         {
//             cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
//             return -1;
//         }

// #ifdef COMPILEDWITHC14
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

//         // Pass the image to the SLAM system
//         SLAM.TrackMonocular(im, tframe);

// #ifdef COMPILEDWITHC14
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

//         double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//         vTimesTrack[ni] = ttrack;

//         // Wait to load the next frame
//         double T=0;
//         if (ni < nImages-1)
//             T = vTimestamps[ni+1] - tframe;
//         else if (ni>0)
//             T = tframe - vTimestamps[ni-1];

//         if (ttrack<T)
//             usleep((T-ttrack)*1e6);
//     }

//     // Stop all threads
//     SLAM.Shutdown();

//     // Tracking time statistics
//     sort(vTimesTrack.begin(), vTimesTrack.end());
//     float totaltime = 0;
//     for(int ni=0; ni<nImages; ni++)
//     {
//         totaltime += vTimesTrack[ni];
//     }
//     cout << "-------" << endl << endl;
//     cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
//     cout << "mean tracking time: " << totaltime/nImages << endl;

//     // Save camera trajectory
//     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

//     return 0;
// }

// bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
// {
//     string strPathTimeFile = strPathToSequence + "/times.txt";
//     string strPrefixLeft = strPathToSequence + "/image_0/";

//     ifstream fTimes;
//     fTimes.open(strPathTimeFile.c_str());

//     if (fTimes.is_open())
//     {
//         cout << "times.txt file is open\n";
//         string line;

//         int i=0;
//         while(getline(fTimes, line))
//         {
//             istringstream iss(line); //put line into stringstream
//             stringstream ss;
//             string word;
//             double time;

//             iss >> word;    // Get the first word of the column

//             ss << line;
//             ss >> time;
//             vTimestamps.push_back(time);
//             vstrImageFilenames.push_back(strPrefixLeft + word + ".jpg");
//             i++;
//         }

//         cout << "Total lines readed: " << i+1 << "\n";
//         return true;
//     }
//     else
//     {
//         cerr << "failed to opencv times.txt\n";
//         return false;
//     }
// }
