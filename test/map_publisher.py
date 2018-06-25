#!/usr/bin/env python
##############################################################################
# Imports
##############################################################################

import numpy as np

import rospy
import nav_msgs.msg as nav_msgs

##############################################################################
# Main
##############################################################################

def map_publisher():
    # publisher
    pub = rospy.Publisher("/map", nav_msgs.OccupancyGrid, queue_size=10, latch=True)

    # msg
    map_msg = nav_msgs.OccupancyGrid()
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"

    map_msg.info.resolution = 0.05
    map_msg.info.width = 200
    map_msg.info.height = 200
    map_msg.info.origin.orientation.w = 1

    # map in middle
    map_msg.info.origin.position.x = -(map_msg.info.width * map_msg.info.resolution / 2)
    map_msg.info.origin.position.y = -(map_msg.info.height * map_msg.info.resolution / 2)

    map_msg.data = np.zeros(map_msg.info.width * map_msg.info.height, dtype=np.uint8)

    pub.publish(map_msg)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node("map_publisher")
    try:
        map_publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
