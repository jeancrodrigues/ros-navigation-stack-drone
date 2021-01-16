#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

def callback(data):
    mp = np.reshape(data.data,(data.info.width, data.info.height))
    im = cv2.cvtColor(np.uint8(mp), cv2.COLOR_GRAY2BGR)    

    # im = cv2.medianBlur(im,5)   

    circles = cv2.HoughCircles(np.uint8(mp),cv2.HOUGH_GRADIENT, 1, 1, param1=4,param2=22, minRadius=10,maxRadius=50)
    print(circles)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(im,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(im,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow("Mapa",im)
    cv2.imwrite("/home/jean/Imagens/dronetrees.jpg",im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def listener():
    print("listener")
    rospy.init_node('python_ros_node')
    rospy.Rate(0.1)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
