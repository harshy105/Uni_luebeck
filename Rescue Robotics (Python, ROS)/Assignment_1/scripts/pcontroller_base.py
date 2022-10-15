#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import tf
import numpy as np
import threading

class controller():
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.update_pose)        
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.k_pose = 1
        self.k_angle = 1
        self.vmax = 2
        self.wmax = 2
        #self.tl = tf.TransformListener()

    def update_pose(self, data): #(self,data_world)
        #update current pose of the robot
        #data = self.tl.transformPose("base_link", data_world)
        self.pose_stamped = data
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(data.pose)

    def get_goal(self):
        x = input("Set your x goal:")
        y = input("Set your y goal:")
        z = input("Set your z goal:")
        angle_deg = input("Set your angle goal in (-180,180):")
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        self.goal_angle = angle_deg*np.pi/180
        print("goal selected")

    def get_goal_cont(self):
        while not rospy.is_shutdown():
            self.get_goal()


    def quaternion_to_euler(self, robot_pose):
        #transforms robot position from quaternion to roll, pitch, yaw
        quaternion = (
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw


    def control_law(self):

        #compute distances in world frame
        dx_world = self.goal_x - self.pose_stamped.pose.position.x
        dy_world = self.goal_y - self.pose_stamped.pose.position.y
        dz_world = self.goal_z - self.pose_stamped.pose.position.z
        dw_world = self.goal_angle - self.yaw

        #compute distances in drone frame
        dx_drone = dx_world*np.cos(self.yaw) + dy_world*np.sin(self.yaw)
        dy_drone = -dx_world*np.sin(self.yaw) + dy_world*np.cos(self.yaw)
        dz_drone = dz_world
        dw_drone = dw_world

    	#p-controller
        ux = self.k_pose*dx_drone
        uy = self.k_pose*dy_drone
        uz = self.k_pose*dz_drone
       	uw = self.k_angle*dw_drone	

       	#saturation of velocities
       	if ux > self.vmax:
       		ux = self.vmax
   		if ux < -self.vmax:
   			ux = -self.vmax

       	if uy > self.vmax:
       		uy = self.vmax
   		if uy < -self.vmax:
   			uy = -self.vmax

       	if uz > self.vmax:
       		uz = self.vmax
   		if uz < -self.vmax:
   			uz = -self.vmax

       	if uw > self.wmax:
       		uw = self.wmax
   		if uw < -self.wmax:
   			uw = -self.wmax

        #return the control values
        return ux,uy,uz,uw


    def move2goal(self):
        while not rospy.is_shutdown():
            ux,uy,uz,uw = self.control_law()
            #define a Twist message here, apply the values from the control lab and publish it to cmd_vel
            message = Twist()

            message.linear.x = ux
            message.linear.y = uy
            message.linear.z = uz
            message.angular.z = uw

            self.vel_publisher.publish(message)

if __name__ == '__main__':
    try:
        rospy.init_node('p_controller')
        p_controller = controller()
        rospy.sleep(1) # waiting for the initial pose update
        p_controller.get_goal()
        t1 = threading.Thread(name='thread1', target=p_controller.get_goal_cont)
        t2 = threading.Thread(name='thread2', target=p_controller.move2goal)
        t1.start()
        t2.start()
    except rospy.ROSInterruptException: pass

