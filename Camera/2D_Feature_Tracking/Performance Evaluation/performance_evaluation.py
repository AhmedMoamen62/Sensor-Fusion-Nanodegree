import os
import subprocess
import csv

os.chdir("./../build")

detector_list = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptor_list = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]
match_list = ["MAT_BF", "MAT_FLANN"]
distance_list = ["DES_BIN", "DES_NBIN"]
selector_list = ["SEL_NN", "SEL_KNN"]

titles1 = ["Detector Type","No. of Keypoints on Preceding Vehicle","Image"]
titles2 = ["Detector Type", "Descriptor Type","No. of Keypoints","Image"]
titles3 = ["Detector Type", "Descriptor Type","Execution Time","Image"]

def task_7():
	par = []
	for detector in detector_list:
		matcher = match_list[0]
		distance = distance_list[0]
		selector = selector_list[0]
		if detector == "SIFT" or detector == "AKAZE":
			distance = distance_list[1]	
		_, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s %s %s %s' % (detector,descriptor,matcher,distance,selector))
		paramters = output_string.split("\n")
		print(paramters)
		par.append(paramters)

	return par

def task_8():
	par = []
	for detector in detector_list:
		for descriptor in descriptor_list:
			if detector != "AKAZE" and descriptor == "AKAZE":
				continue
			matcher = match_list[0]
			distance = distance_list[0]
			selector = selector_list[0]
			if detector == "SIFT" or detector == "AKAZE" or descriptor == "SIFT":
				distance = distance_list[1]			
			_, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s %s %s %s' % (detector,descriptor,matcher,distance,selector))
			paramters = output_string.split("\n")
			print(paramters)
			par.append(paramters)
			
	return par

def task_9():
	par = []
	for detector in detector_list:
		for descriptor in descriptor_list:
			if detector != "AKAZE" and descriptor == "AKAZE":
				continue
			matcher = match_list[0]
			distance = distance_list[0]
			selector = selector_list[0]
			if detector == "SIFT" or detector == "AKAZE" or descriptor == "SIFT":
				distance = distance_list[1]			
			_, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s %s %s %s' % (detector,descriptor,matcher,distance,selector))
			paramters = output_string.split("\n")
			print(paramters)
			par.append(paramters)
			
	return par

task_7_file = '../Performance Evaluation/Keypoints_in_ROI.csv'
task_8_file = '../Performance Evaluation/Matched_keypoints.csv'
task_9_file = '../Performance Evaluation/Execution_Time.csv'

with open(task_9_file, mode='w') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    csv_writer.writerow(titles3)
    par = task_9()
    for type_des in par:
    	for image in type_des:
    		csv_writer.writerow(image.split())