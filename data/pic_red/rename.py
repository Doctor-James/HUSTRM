from cv2 import cv2
import os
import sys

#print(sys.argv[0], sys.argv[1])

root = './'
class_ = int(sys.argv[1])

dir_path = root+"tmp"
dir_folder = os.listdir(dir_path)
id_ = int(sys.argv[2])
for file_ in dir_folder:
	f = dir_path + '/' + file_
	img = cv2.imread(f)
	cv2.imwrite(dir_path + '/../' + str(class_) + '/' + str(id_) + '.png', img)
	#print(f)
	id_ += 1
	os.remove(f)
