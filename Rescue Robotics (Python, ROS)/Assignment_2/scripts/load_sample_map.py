#!/usr/bin/env python
"""
@author: Lars Schilling

"""
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2


def transform_data(data):

	data=np.array(data)
	data[data==0] = 1
	data[data==127] = 2
	data[data==255] = 0

	return data



if __name__ == '__main__':
	rospack = rospkg.RosPack()
	img = Image.open(rospack.get_path('heron_exploration')+'/sample_maps/sample_map_5.png')
	img_work = transform_data(img)

	###try frontier point detection with img_work
	#find all frontier points, can be defined as edge detection problem, cv2.Canny can be used
	img_edge = cv2.Canny(img_work,7.9,8)
	frontier_all = np.where(img_edge==255)

	fig1, (ax1,ax2) = plt.subplots(1,2)
	ax1.imshow(img_edge)
	ax2.scatter(frontier_all[1],frontier_all[0])
	ax2.invert_yaxis()
	plt.xlim(0,64)
	plt.title("Frontier Points")
	plt.show()

	#calculate information gain, tip: set everything to zero expect unexplored area
	#you can use cv2.filter2D with your own kernel
	img_work[img_work==1] = 0
	kernel = np.ones((5,5))
	img_info = cv2.filter2D(img_work,-1,kernel)
	#fig2 = plt.figure()
	#plt.imshow(img_info)
	#plt.show()

	#find the frontier point with the biggest information gain, this will be our goal point	
	frontier_max = np.unravel_index(img_info.argmax(), img_info.shape)
	print("The frontier point with the biggest information gain is", frontier_max)