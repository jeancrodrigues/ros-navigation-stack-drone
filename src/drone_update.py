#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
import cv2

mp = None

def callbackupdate(data):  
    global mp  
    print("update")
    
    print(len(data.data))
    print(data.width)
    print(data.height)

    mp_update = np.reshape(data.data,(data.width, data.height))

    print(mp is None)

    if mp is not None:
        # for i in range(data.width):
        #     for j in range(data.height):
        #         mp[data.x + i][data.y + j] = mp_update[i][j]

        im = cv2.cvtColor(np.uint8(mp_update), cv2.COLOR_GRAY2BGR)    
        circles = cv2.HoughCircles(np.uint8(mp_update),cv2.HOUGH_GRADIENT, 1, 1, param1=50,param2=30, minRadius=10,maxRadius=50)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            print(circles.shape)
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(im,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(im,(i[0],i[1]),2,(0,0,255),3)
            
        cv2.imshow("Image",im)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def callback(data):
    global mp
    print("callback")
    mp = np.reshape(data.data,(data.info.width, data.info.height))

def listener():
    print("listener")
    rospy.init_node('python_ros_node')
    rospy.Rate(0.1)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback)
    rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, callbackupdate)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
