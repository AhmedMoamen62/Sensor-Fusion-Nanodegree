### Performance Evaluation

<img src="../images/Image Evaluation/SHITOMASI_BRISK_TTC_18.png" width="820" height="248" />

This folder contains implementation of **Time-To-Collision** (TTC) using **LIDAR** and **Camera** of different keypoints detectors and descriptors extractor, compare different approach according to the best Evaluated Time of the preceding vehicle.

## Implemented keypoints Detector

	SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

## Implemented Descriptors Extractor

	BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

# 1. Match 3D Objects

Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

```
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // get numbers of boxes in previous and current frames
    int prevBBNumber = prevFrame.boundingBoxes.size();
    int currBBNumber = currFrame.boundingBoxes.size();
    // initialize 2D vector of numbers of boxes with zeros to count the max keypoints number corresponding to box in previous and current frame
    vector<vector<int>> prevVscurrCount(prevBBNumber,vector<int>(currBBNumber,0));

    // iterate over all matches between previous and current frame
    for(auto match = matches.begin(); match != matches.end(); ++match)
    {
        // extract the keypoints of the two frames
        cv::KeyPoint kptPrev = prevFrame.keypoints[match->queryIdx];
        cv::KeyPoint kptCurr = currFrame.keypoints[match->trainIdx];

        // iterate over all previous boxes
        for(auto prevBox = prevFrame.boundingBoxes.begin(); prevBox != prevFrame.boundingBoxes.end(); ++prevBox)
        {
            // check if the previous keypoint in the box
            if(prevBox->roi.contains(kptPrev.pt))
            {
                // iterate over all current boxes
                for(auto currBox = currFrame.boundingBoxes.begin(); currBox != currFrame.boundingBoxes.end(); ++currBox)
                {
                    // check if the current keypoint in the box
                    if(currBox->roi.contains(kptCurr.pt))
                    {
                        // increase the numbers of points
                        prevVscurrCount[prevBox->boxID][currBox->boxID]++;
                    }
                }
            }
        }
    }

    // iterate over all boxes in previous frame
    for(int prevBoxID = 0 ; prevBoxID < prevVscurrCount.size(); prevBoxID++)
    {
        // get the max index correspoing keypoint in box in current frame
        int maxCurrentBoxID = max_element(prevVscurrCount[prevBoxID].begin(),prevVscurrCount[prevBoxID].end()) - prevVscurrCount[prevBoxID].begin();
        // check if there's no keypoints then insert the boxID for previous and current frame
        if(prevVscurrCount[prevBoxID][maxCurrentBoxID] != 0)
            bbBestMatches.insert(pair<int,int>(prevBoxID,maxCurrentBoxID));
    }
}
```

# 2. Compute Lidar-based TTC

Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

```
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1 / frameRate;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane
    TTC = 0.0;

    // find closest distance to Lidar points within ego lane
    vector<double> xPrev,xCurr;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if(fabs(it->y) <= laneWidth / 2.0)
            xPrev.push_back(it->x);
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if(fabs(it->y) <= laneWidth / 2.0)
            xCurr.push_back(it->x);
    }

    // get the average of the distances
    double minXPrev = 0.0;
    double minXCurr = 0.0;

    if(xPrev.size() > 0 && xCurr.size() > 0)
    {
        minXPrev = accumulate(xPrev.begin(),xPrev.end(),0.0)/xPrev.size();
        minXCurr = accumulate(xCurr.begin(),xCurr.end(),0.0)/xCurr.size();
    }

    // compute TTC from both measurements
    if(minXPrev - minXCurr != 0.0)
        TTC = minXCurr * dT / (minXPrev - minXCurr);
}
```

# 3. Associate Keypoint Correspondences with Bounding Boxes

Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

```
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // store all matches that are in the current bounding box
    vector<cv::DMatch> kptMatches_ROI;
    // store the average distance for all matches in the current bounding box
    double mean_distance = 0.0;
    // iterate over all matches
    for(auto match = kptMatches.begin(); match != kptMatches.end(); ++match)
    {
        // check if the current keypoint in the current bounding box
        if(boundingBox.roi.contains(kptsCurr[match->trainIdx].pt))
        {
            // add it to the ROI vector and increase the mean_distance
            kptMatches_ROI.push_back(*match);
            mean_distance += match->distance;
        }
    }
    // if there's no matches, return from the function
    if(kptMatches_ROI.size() == 0)
        return;
    // get the mean of all euclidean distances between keypoint matches
    mean_distance /= kptMatches_ROI.size();

    // store matches which are less than 75% of the mean distance
    for(auto match = kptMatches_ROI.begin(); match != kptMatches_ROI.end(); ++match)
    {
        if(match->distance < 0.75 * mean_distance)
            boundingBox.kptMatches.push_back(*match);
    }
}
```

# 4. Compute Camera-based TTC

Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

```
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // only continue if list of matches is not empty
    if (kptMatches.size() == 0)
    {
        TTC = 0.0;
        return;
    }

    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = 0.0;
        return;
    }

    // compute camera-based TTC from distance ratios
    sort(distRatios.begin(),distRatios.end());
    double medianDistRatio = distRatios.size() % 2 == 0 ? (distRatios[(distRatios.size()-1)/2] + distRatios[(distRatios.size()+1)/2])/2 : distRatios[distRatios.size()/2];

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);
}
```

# Performance Evaluation 1
	
Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

Some examples with wrong TTC estimate of the Lidar sensor:

| Image | Description |
| :-------: | :----------------------------: |
| <img src="../images/Image Evaluation/SHITOMASI_BRISK_TTC_3.png" width="320" height="100" /> | TTC from Lidar is not correct because of some outliers. Need to filter outliers |
| <img src="../images/Image Evaluation/SHITOMASI_BRISK_TTC_4.png" width="320" height="100" /> | TTC from Lidar is not correct because of some outliers. Need to filter outliers |
| <img src="../images/Image Evaluation/SHITOMASI_BRISK_TTC_8.png" width="320" height="100" /> | TTC from Lidar is not correct because of some outliers. Need to filter outliers |

# Performance Evaluation 2 

Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

Some examples with wrong TTC estimate of the Camera: [TTC](TTC.csv)

| Image | Description |
| :-------: | :----------------------------: |
| <img src="../images/Image Evaluation/HARRIS_BRISK_TTC_3.png" width="320" height="100" /> | TTC from Camera is negative because estimation uses median distance ration and if it is less than 1 it will be negative |
| <img src="../images/Image Evaluation/HARRIS_BRIEF_TTC_18.png" width="320" height="100" /> | TTC from Camera is not correct because of some outliers. Need to filter outliers |
| <img src="../images/Image Evaluation/HARRIS_BRISK_TTC_10.png" width="320" height="100" /> | TTC from Camera is negative because estimation uses median distance ration and if it equal to 1 it will be divided by zero |

The TOP 3 Detector/Descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles are: 

1. SHITOMASI-BRISK
2. SHITOMASI-BRIEF
3. SHITOMASI-ORB