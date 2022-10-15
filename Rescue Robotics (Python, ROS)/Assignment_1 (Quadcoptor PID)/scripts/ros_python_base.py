#!/usr/bin/env python
import rospy

if __name__ == '__main__':	
    try:
        print("test")
    except rospy.ROSInterruptException: pass
