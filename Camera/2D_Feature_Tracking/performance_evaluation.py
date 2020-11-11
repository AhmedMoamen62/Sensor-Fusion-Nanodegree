import os
import subprocess
import csv

os.chdir("./build")

detector_list = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptor_list = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]

titles1 = ["Detector Type","No. of Keypoints on Preceding Vehicle","Image"]
titles2 = ["Detector Type", "Descriptor Type","No. of Keypoints","Image"]
titles3 = ["Detector Type", "Descriptor Type","Execution Time","Image"]

# cnt = 1
# final_string_list = []
# for detector in detector_list:
# 	for descriptor in descriptor_list:
# 		state, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s' % (detector, descriptor))
# 		final_string_list.append(str(cnt) + "\n" + output_string+"\n")
# 		cnt += 1
# 		print(output_string)

def task_7():
	par = []
	for detector in detector_list:
		_, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s' % (detector))
		paramters = output_string.split("\n")
		print(paramters)
		par.append(paramters)

	return par

def task_8():
	par = []
	for detector in detector_list:
		for descriptor in descriptor_list:
			_, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s' % (detector,descriptor))
			paramters = output_string.split("\n")
			print(paramters)
			par.append(paramters)
			
	return par

def task_9():
	par = []
	for detector in detector_list:
		for descriptor in descriptor_list:
			_, output_string = subprocess.getstatusoutput('./2D_feature_tracking %s %s' % (detector,descriptor))
			paramters = output_string.split("\n")
			print(paramters)
			par.append(paramters)
			
	return par

task_7_file = '../Output.csv'
task_8_file = '../Matching.csv'
task_9_file = '../Time.csv'

with open(task_9_file, mode='w') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    csv_writer.writerow(titles3)
    par = task_9()
    for type_des in par:
    	for image in type_des:
    		csv_writer.writerow(image.split())