/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

namespace matdes_params // Matching descriptors parameters
{
    std::string MATCHER_BF = "MAT_BF";
    std::string MATCHER_FLANN = "MAT_FLANN";
    std::string DESC_BINARY = "DES_BIN";
    std::string DESC_NOT_BINARY = "DES_NBIN";
    std::string SELECT_NN = "SEL_NN";
    std::string SELECT_KNN = "SEL_KNN";
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    std::string detectorType_argv;
    std::string descriptorType_argv;
    std::string matcherType_argv;
    std::string distanceType_argv;
    std::string selectorType_argv;

    // Init vars for stats script
    if(argc == 6)
    {
        detectorType_argv = argv[1];
        descriptorType_argv = argv[2];
        matcherType_argv = argv[3];
        distanceType_argv = argv[4];
        selectorType_argv = argv[5];
    }
    else if (argc == 5)
    {
        detectorType_argv = argv[1];
        descriptorType_argv = argv[2];
        matcherType_argv = argv[3];
        distanceType_argv = argv[4];
        selectorType_argv = matdes_params::SELECT_NN;
    }
    else if (argc == 4)
    {
        detectorType_argv = argv[1];
        descriptorType_argv = argv[2];
        matcherType_argv = argv[3];
        distanceType_argv = matdes_params::DESC_BINARY;
        selectorType_argv = matdes_params::SELECT_NN;
    }
    else if (argc == 3)
    {
        detectorType_argv = argv[1];
        descriptorType_argv = argv[2];
        matcherType_argv = matdes_params::MATCHER_BF;
        distanceType_argv = matdes_params::DESC_BINARY;
        selectorType_argv = matdes_params::SELECT_NN;
    }
    else if (argc == 2)
    {
        detectorType_argv = argv[1];
        descriptorType_argv = "BRIEF";
        matcherType_argv = matdes_params::MATCHER_BF;
        distanceType_argv = matdes_params::DESC_BINARY;
        selectorType_argv = matdes_params::SELECT_NN;
    }
    else if (argc == 1)
    {
        detectorType_argv = "SHITOMASI";
        descriptorType_argv = "BRIEF";
        matcherType_argv = matdes_params::MATCHER_BF;
        distanceType_argv = matdes_params::DESC_BINARY;
        selectorType_argv = matdes_params::SELECT_NN;
    }

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        if(dataBuffer.size() == dataBufferSize)
            dataBuffer.erase(dataBuffer.begin());
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = detectorType_argv;

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        double total_execution_time;
        double t = (double)cv::getTickCount();
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, bVis);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, bVis);
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        total_execution_time = 1000 * t / 1.0;
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = false;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            for(auto it = keypoints.begin() ; it != keypoints.end() ; ++it)
            {
                if(!vehicleRect.contains(it->pt))
                {
                    keypoints.erase(it);
                    --it;
                }
            }
            //cout << detectorType << " " << keypoints.size() << " " << imgNumber.str() + imgFileType << endl;
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            //cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        //cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = descriptorType_argv; // BRIEF, ORB, FREAK, AKAZE, SIFT
        t = (double)cv::getTickCount();
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        total_execution_time += 1000 * t / 1.0;
        cout << detectorType_argv << " " << descriptorType_argv << " " << total_execution_time << " " << imgNumber.str() + imgFileType << endl;
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = matcherType_argv;        // MAT_BF, MAT_FLANN
            string descriptorType = distanceType_argv;    // DES_BINARY, DES_HOG
            string selectorType = selectorType_argv;      // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                //cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
            //cout << detectorType_argv << " " << descriptorType_argv << " " << matches.size() << " " << imgNumber.str() + imgFileType << endl;
        }
    } // eof loop over all images

    return 0;
}
