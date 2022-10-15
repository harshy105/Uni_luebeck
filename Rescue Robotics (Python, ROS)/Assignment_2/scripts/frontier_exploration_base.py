#!/usr/bin/env python
"""
@author: Lars Schilling

"""
#imports
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt


def get_data():
	#get map and metadata to transfrom into a matrix

	msg=rospy.wait_for_message('/map_image/tile', Image)
	odom=rospy.wait_for_message('/odometry/gps', Odometry)
	#odom=rospy.wait_for_message('/pose_gt', Odometry)
	metadata=rospy.wait_for_message('/map_metadata', MapMetaData)
	resolution=metadata.resolution

	bridge = CvBridge()
	data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	data=np.array(data)
	data[data==0] = 1
	data[data==127] = 2
	data[data==255] = 0

	return data, odom, resolution

def frontier_exploration():

	img_work, odom, resolution = get_data()

	#visualize img_work
	#print(img_work)
	#plt.imshow(img_work)
	#plt.show()

	#find all frontier points, can be defined as edge detection problem, cv2.Canny can be used
	img_edge = cv2.Canny(img_work,7.9,8)
	frontier_all = np.where(img_edge==255)

	#fig1, (ax1,ax2) = plt.subplots(1,2)
	#ax1.imshow(img_edge)
	#ax2.scatter(frontier_all[1],frontier_all[0])
	#ax2.invert_yaxis()
	#plt.xlim(0,64)
	#plt.title("Frontier Points")
	#plt.show()

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
	#print("The frontier point with the biggest information gain is", frontier_max)

	#define a PoseStamped message here and publish it on the move_base_publisher
	goal=PoseStamped()
	goal.header.stamp=rospy.Time.now()
	goal.header.frame_id="odom"
	goal.pose.orientation.w=1
	#define x and y position of the pose here
	#use the odom position and the goal point
	#reminder: the "odom position of the boat" is the center of the image!
	#the goal point must still be converted, you will need the resolution of the map for this which is given with the parameter "resolution"
	delta_x = resolution*(frontier_max[1]-32)
	delta_y = resolution*(frontier_max[0]-32)
	goal.pose.position.x = odom.pose.pose.position.x + delta_x
	goal.pose.position.y = odom.pose.pose.position.y + delta_y

	move_base_publisher.publish(goal)

if __name__ == '__main__':
	try:

		rospy.init_node('frontier_exploration')
		move_base_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		while not rospy.is_shutdown():
			frontier_exploration()
	except rospy.ROSInterruptException: pass
