import os
import subprocess
import csv

os.chdir("./../build")

detector_list = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptor_list = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]
match_list = ["MAT_BF", "MAT_FLANN"]
distance_list = ["DES_BIN", "DES_NBIN"]
selector_list = ["SEL_NN", "SEL_KNN"]

titles = ["Detector Type", "Descriptor Type","Image", "TTC Lidar", "TTC Camera", "TTC Absolute Difference"]


def task():
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
			_, output_string = subprocess.getstatusoutput('./3D_object_tracking %s %s %s %s %s' % (detector,descriptor,matcher,distance,selector))
			paramters = output_string.split("\n")
			print(paramters)
			par.append(paramters)
			
	return par

task_file = '../Performance Evaluation/TTC.csv'


with open(task_file, mode='w') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    csv_writer.writerow(titles)
    par = task()
    for type_des in par:
    	for image in type_des:
    		csv_writer.writerow(image.split())