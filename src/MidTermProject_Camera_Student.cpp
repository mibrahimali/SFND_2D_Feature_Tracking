/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

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
    bool bVis = true;            // visualize results

    // Pipeline Algorithms
    string detectorType = "FAST"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType = "BRIEF"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    // Statistical Analysis Variables
    vector<double> detector_runtimes;
    vector<double> descriptor_runtimes;
    vector<size_t> total_detected_keypoints;
    vector<size_t> filtered_keypoints_count;
    vector<size_t> matches_count;
    
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
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        // check size of dataBuffer
        if(dataBuffer.size() > dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }
        //// EOF STUDENT ASSIGNMENT
        // cout << "#1 : LOAD IMAGE INTO BUFFER done , buffer size = "<<dataBuffer.size() << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType

        double runtime;
        if (detectorType.compare("SHITOMASI") == 0)
        {
            runtime = detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            runtime = detKeypointsHarris(keypoints,imgGray,bVis);
        }
        else
        {
            runtime = detKeypointsModern(keypoints,imgGray,detectorType,bVis); // please note the function have internal error check for unimplemented detectors
        }
        detector_runtimes.push_back(runtime);
        total_detected_keypoints.push_back(keypoints.size());
        
        // cv::Mat visImage = img.clone();
        // cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::imwrite( imgNumber.str()+"keypoint.png", visImage );
        
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        
        // cv::Mat roi_img=img.clone();
        // rectangle(roi_img,vehicleRect,cv::Scalar(255,0,0),3,8,0);
        // cv::imwrite( imgNumber.str()+"ROI.png", roi_img );
        
        if (bFocusOnVehicle)
        {
            std::vector<cv::KeyPoint> filteredKeypoints;
            // std::cout<<"Focusing Only on ROI for Preciding Vehicle , other Keypoint will be erased"<<std::endl;
            for (cv::KeyPoint keypoint : keypoints)
            {
                if (vehicleRect.contains(keypoint.pt))
                    filteredKeypoints.push_back(keypoint);
            }
            keypoints = filteredKeypoints;
        }
        filtered_keypoints_count.push_back(keypoints.size());
        
        // detected features neighboorhood distribution analysis
        double mean = std::accumulate(keypoints.begin(), keypoints.end(), 0.0,
            [](double sum, const cv::KeyPoint& i) { return sum + i.size ;}) /keypoints.size();
        auto add_square = [mean](double sum, const cv::KeyPoint& i)
        {
            auto d = i.size - mean;
            return sum + d*d;
        };
        double total = std::accumulate(keypoints.begin(), keypoints.end(), 0.0, add_square);
        double variance = total / keypoints.size();

        // std::cout<< detectorType<<" Detector detect keypoints with mean "<<mean<<"and var "<< variance<<endl;
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
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        // cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType

        cv::Mat descriptors; 
        runtime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;
        descriptor_runtimes.push_back(runtime);
        // cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            // string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
            string descriptorFamily = (descriptorType.compare("SIFT") == 0 )? "DES_HOG":"DES_BINARY"; // DES_BINARY, DES_HOG
            // string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorFamily, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            matches_count.push_back(matches.size());
            // cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
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
                // cv::imwrite( imgNumber.str()+"matching.png", matchImg );
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }

    } // eof loop over all images

    // Run Statistical Analysis on  implemented pipeline
    double average_detector_runtime = std::accumulate(detector_runtimes.begin(),detector_runtimes.end(),0.0)/detector_runtimes.size();
    double average_detected_keypoints = std::accumulate(total_detected_keypoints.begin(),total_detected_keypoints.end(),0)/total_detected_keypoints.size();
    double average_detected_keypoints_inbox = std::accumulate(filtered_keypoints_count.begin(),filtered_keypoints_count.end(),0)/filtered_keypoints_count.size();
    
    double average_descriptor_runtime = std::accumulate(descriptor_runtimes.begin(),descriptor_runtimes.end(),0.0)/descriptor_runtimes.size();
    double average_matches_count = std::accumulate(matches_count.begin(),matches_count.end(),0)/matches_count.size();
    std::cout<< detectorType<<" Detector run for "<<detector_runtimes.size()<< " with Avg. runtime "<<average_detector_runtime<<" ms"<<endl;
    std::cout<< " Avg. Detected Keypoints "<< average_detected_keypoints << " and " << average_detected_keypoints_inbox << " are in ROI" << endl;
    std::cout<< descriptorType<<" Descriptor run  "<< " with Avg. runtime "<<average_descriptor_runtime<<" ms"<<endl;
    std::cout<< "Average number of matches found = "<<average_matches_count<<endl;
    return 0;
}
