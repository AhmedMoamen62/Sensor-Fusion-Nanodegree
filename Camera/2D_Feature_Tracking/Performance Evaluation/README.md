## Performance Evaluation

<img src="../images/keypoints.png" width="820" height="248" />

This folder contains implementation of different keypoints detectors and descriptors extractor, compare different approach according to number of detected keypoints and execution time to detect keypoints and extract descriptors to estimate
the motion and position of the preceding vehicle in **2D** then use it to evaluate time to collision **TTC**.

# Implemented keypoints Detector

	SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

# Implemented Descriptors Extractor

	BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

We will compare combinations of them according to 3 **Metrics**

# 1. Average Keypoints in ROI for 10 Images 
	
- Applied on BF Matching, NN Select and Binary Descriptors. 
- Detailed keypoitns in ROI for every image in [Keypoints in ROI](Keypoints_in_ROI.csv)

| Detectors | Keypoints in Preceding Vehicle |
| :-------: | :----------------------------: |
| SHITOMASI | 117.9                     	 |
| HARRIS    | 24.8                        	 |
| FAST      | 149.1                    		 |
| BRISK     | 276.2                  		 |
| ORB       | 116.1                     	 |
| AKAZE     | 167                            |
| SIFT      | 138.6            		      	 |

# 2. Average Matched Keypoints between 10 Images 

- Applied on BF Matching, NN Select for all and Non-Binary Descriptors for AKAZE, SIFT and  Binary Descriptors for the rest.
- Detailed Matched Keypoints for sequence of 2 images in [Matched keypoints](Matched_keypoints.csv)

| Detectors/Descriptors |  BRISK  |  BRIEF  |   ORB   |  FREAK  | AKAZE  |  SIFT   |
| :-------------------: |  :---:  |  :---:  |   :-:   |  :---:  | :---:  |  :--:   |
| SHITOMASI             | 717.67  | 976.44  | 890.67  | 717.55  | N/A    | 1046.89 |
| HARRIS                | 83.89   | 107.11  | 99.33   | 83.67   | N/A    | 111     |
| FAST                  | 971.33  | 1251.22 | 1165.67 | 919.33  | N/A    | 1373.22 |
| BRISK                 | 1426.67 | 1671.44 | 1391.78 | 1323.44 | N/A    | 1637.89 |
| ORB                   | 306.56  | 246.11  | 336.78  | 336.78  | N/A    | 124.89  |
| AKAZE                 | 371.22  | 371.22  | 888.11  | 371.22  | 915.89 | 985.67  |
| SIFT                  | 888.11  | 870.67  | N/A     | 1023.67 | N/A    | 658.11  |

# 3. Average Execution Time in ms for 10 Images 

- Applied on BF Matching, NN Select for all and Non-Binary Descriptors for AKAZE, SIFT and  Binary Descriptors for the rest.
- Detailed Execution Time for every 2 image in [Execution Time](Execution_Time.csv)

| Detectors/Descriptors |  BRISK  |  BRIEF  |   ORB   |  FREAK  | AKAZE   |   SIFT   |
| :-------------------: |  :---:  |  :---:  |   :-:   |  :---:  | :---:   |   :--:   |
| SHITOMASI             | 2102.88 | 121.99  | 171.91  | 377.23  | N/A     | 878.92   |
| HARRIS                | 2057.22 | 107.55  | 156.99  | 345.33  | N/A     | 816.91   |
| FAST                  | 834.02  | 67.92   | 117.42  | 187.33  | N/A     | 1145.76  |
| BRISK                 | 1733.69 | 946.72  | 1128.56 | 1066.65 | N/A     | 3183.03  |
| ORB                   | 1085.47 | 212.82  | 411.48  | 303.93  | N/A     | 1763.94  |
| AKAZE                 | 1998.39 | 1226.76 | 1344.51 | 1338.61 | 2377.47 | 2283.78  |
| SIFT                  | 4171.95 | 3536.83 | N/A     | 3606.48 | N/A     | 6869.474 |

From the previous tables we find BRISK Detector has the higher average keypoint for the preceding vehicle
and BRISK-BRIEF Detectors/Descriptors has th higher average keypoint for image

Comparing run-time execution we will find top 3 pair of Detectors/Descriptors as following :

1. FAST-BRIEF
2. HARRIS-BRIEF
3. FAST-ORB