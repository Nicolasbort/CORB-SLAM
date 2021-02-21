//
// Created by lifu on 2017/6/4.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <thread>

#include "MapFusion.h"
#include "ServerMap.h"

using namespace std;

using namespace CORBSLAM_SERVER;

void help();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Corbslam_server");
    ros::start();

    if( argc != 3) {
        help();
        return false;
    }

    ROS_INFO("Corbslam_server start!");

    // Create MapFusion system. It initializes all system threads and gets ready to process frames.

    string strSettingPath = argv[2];

    MapFusion* mapfusion = new MapFusion( strSettingPath );

    mapfusion->loadORBVocabulary( argv[1] );
    mapfusion->createKeyFrameDatabase();

    ros::NodeHandle n;

    // Here the service is created and advertised over ROS
    ros::ServiceServer InsertKeyFrameService = n.advertiseService("insertKeyFrameToMap", &MapFusion::insertKeyFrameToMap, mapfusion );
    ros::ServiceServer InsertMapPointService = n.advertiseService("insertMapPointToMap", &MapFusion::insertMapPointToMap, mapfusion);
    ros::ServiceServer updateKeyFrameToMapService = n.advertiseService("updateKeyFrameToMap", &MapFusion::updateKeyFrameToMap, mapfusion);
    ros::ServiceServer updateMapPointToMapService = n.advertiseService("updateMapPointToMap", &MapFusion::updateMapPointToMap, mapfusion);

    ROS_INFO("Publish services finished !");

    thread * mapFuisonThread = new thread(&MapFusion::fuseSubMapToMap, mapfusion);

    // publish update keyframe and mappoint poses topic
    thread * pubThread = new thread(&MapFusion::runPubTopic, mapfusion);

    // wait to get subcribe new keyframes or new mappoints
    ros::MultiThreadedSpinner spinner(2);

    spinner.spin();

    // Save camera trajectory

    ros::shutdown();

    delete mapfusion;
    delete mapFuisonThread;
    delete pubThread;

    return 0;
}


void help()
{
    cout << endl <<
    "Don't forget to run roscore in another terminal before run this one!" << endl <<
    "ROS Usage: rosrun corbslam_server corbslam_server <PATH_TO_VOCABULARY.txt> <PATH_TO_CONFIG.yaml>"
    << endl << endl;
}