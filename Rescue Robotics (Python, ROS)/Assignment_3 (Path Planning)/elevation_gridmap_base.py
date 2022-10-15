# -*- coding: utf-8 -*-
"""
@author: Lars Schilling

"""
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt

#add imports for the pathfinding package
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

def callback(msg):
        print('Recieved map data')
        bridge = CvBridge()
        data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height_map=np.array(data, dtype=np.uint)
        # add a nonlinearity to the height map to create a height cost map (low points very low cost, high points very high cost)
        height_cost_map = np.square(height_map)

        # create a distance cost map with the shape of height_costmap
        # pixels around the center should have low cost and get higher at the edges
        x, y = np.ogrid[0:60, 0:60]
        distances_cost_map = np.sqrt((x - 30) ** 2 + (y - 30) ** 2)

        # define a combined cost map based height and distance cost map
        # this could be a sum or multiplication
        combined_cost_map = 2*height_cost_map + distances_cost_map + 1

        # implement the AStarFinder from the pathfinding package to the combined cost map
        # to find a path from the center of the image (30,30) to the upper edge of the image (30,0)
        grid = Grid(matrix=combined_cost_map)
        start = grid.node(30, 30)
        end = grid.node(30, 0)
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        path_plot = np.zeros((60, 60))
        for m in path:
            path_plot[m[1], m[0]] = 1

        # plot your height, distance and combined cost map, as well as the astar path

        # plt.imshow(path_plot)
        # plt.imshow(height_map, alpha=0.7)
        # plt.show()

        plt.subplot(2, 2, 1)
        plt.imshow(height_map)
        plt.title("height_map")
        plt.subplot(2, 2, 2)
        plt.imshow(distances_cost_map)
        plt.title("distance_cost_map")
        plt.subplot(2, 2, 3)
        plt.imshow(combined_cost_map)
        plt.title("combined_cost_map")
        plt.subplot(2, 2, 4)
        plt.imshow(path_plot)
        plt.title("path_plot")
        plt.tight_layout(pad=0.3)
        plt.show()
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:

        rospy.init_node('elevation_path')
        sub=rospy.Subscriber("/grid_map_image", Image, callback, queue_size=1)
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException: pass
