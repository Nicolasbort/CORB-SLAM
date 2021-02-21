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



#include "System.h"   // IWYU pragma: associated

#include <pangolin/pangolin.h>
#include <unistd.h>
#include <stdlib.h>
#include <thread>
#include <iomanip>
#include <algorithm>
#include <iostream>
#include <list>

#include "Converter.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "Viewer.h"

namespace ORB_SLAM2
{
class MapPoint;

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, const int clientId) : mSensor(sensor), mbReset(false),
                                                                mbActivateLocalizationMode(false),
                                                                mbDeactivateLocalizationMode(false) {
        // Output welcome message
        cout << endl <<
             "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
             "This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
             "This is free software, and you are welcome to redistribute it" << endl <<
             "under certain conditions. See LICENSE.txt." << endl << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl;
        else if (mSensor == STEREO)
            cout << "Stereo" << endl;
        else if (mSensor == RGBD)
            cout << "RGB-D" << endl;

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        mpCacher = new Cache();

        mpCacher->pClientId = clientId;

        mpCacher->loadORBVocabulary(strVocFile);

        mpCacher->createKeyFrameDatabase();

        mpCacher->createMap();

        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpCacher->getMpMap());
        mpMapDrawer = new MapDrawer(mpCacher->getMpMap(), strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpCacher, mpFrameDrawer, mpMapDrawer,
                                 strSettingsFile, mSensor);

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpCacher, mSensor == MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        mptCacheUpdate = new thread(&ORB_SLAM2::Cache::runUpdateToServer, mpCacher);

        mptCacheSub = new thread(&ORB_SLAM2::Cache::runSubFromServer, mpCacher);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpCacher, mSensor != MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

        //Initialize the Viewer thread and launch
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        if (bUseViewer)
            mptViewer = new thread(&Viewer::Run, mpViewer);

        mpTracker->SetViewer(mpViewer);

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) {
        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        return mpTracker->GrabImageStereo(imLeft, imRight, timestamp);
    }

    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp) {
        if (mSensor != RGBD) {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        return mpTracker->GrabImageRGBD(im, depthmap, timestamp);
    }

    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) {
        if (mSensor != MONOCULAR) {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        return mpTracker->GrabImageMonocular(im, timestamp);
    }

    void System::ActivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    void System::Reset() {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown() {

        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        // mpViewer->RequestFinish();

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
               !mpViewer->isFinished() || mpLoopCloser->isRunningGBA()) {
            usleep(5000);
        }

        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

    void System::SaveTrajectoryTUM(const string &filename) {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR) {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpCacher->getAllKeyFramesInMap();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        // list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        // list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        // list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        // for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
        //              lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++) {
        for (list<Tracking::TrackedFrame>::iterator iter = mpTracker->tracked_frames.begin(), iter_end = mpTracker->tracked_frames.end(); iter != iter_end; iter++)
        {
            // if (*lbL)
            if(iter->lost)
                continue;

            // KeyFrame *pKF = *lRit;
            KeyFrame* pKF = iter->reference_keyframe;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad()) {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            // cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Tcw = iter->relative_frame_pose*Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            // f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
            //   << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            f << setprecision(6) << iter->time << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;
    }


    void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
        cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpCacher->getAllKeyFramesInMap();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        //cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
              << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }

        f.close();
        cout << endl << "trajectory saved!" << endl;

        cout << "Load mp" << endl;

    }

    void System::SaveTrajectoryKITTI(const string &filename) {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR) {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpCacher->getAllKeyFramesInMap();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        // list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        // list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        // for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end();
        //      lit != lend; lit++, lRit++, lT++) {
        for (list<Tracking::TrackedFrame>::iterator iter = mpTracker->tracked_frames.begin(), iter_end = mpTracker->tracked_frames.end(); iter != iter_end; iter++)
        {
            // ORB_SLAM2::KeyFrame *pKF = *lRit;
            ORB_SLAM2::KeyFrame* pKF = iter->reference_keyframe;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            while (pKF->isBad()) {
                //  cout << "bad parent" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            // cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Tcw = iter->relative_frame_pose*Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2)
              << " " << twc.at<float>(0) << " " <<
              Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1)
              << " " <<
              Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2)
              << endl;
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;
    }

    void System::SaveMap(const string &filename) {
        this->mpCacher->SaveMap(filename);
    }

    void System::LoadMap(const string &filename) {
        this->mpCacher->LoadMap(filename);
    }

} //namespace ORB_SLAM