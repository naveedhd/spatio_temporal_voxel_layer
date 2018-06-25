#!/usr/bin/env python
##############################################################################
# Imports
##############################################################################

import numpy as np

import rospy
import sensor_msgs.msg as sensor_msgs

##############################################################################
# Main
##############################################################################

def point_cloud_publisher():
    # publisher
    pub = rospy.Publisher("/camera/depth/points", sensor_msgs.PointCloud2, queue_size=10)

    # msg
    point_cloud = sensor_msgs.PointCloud2()
    point_cloud.header.stamp = rospy.Time.now()
    point_cloud.header.frame_id = "camera_link"

    point_cloud.fields = [
        sensor_msgs.PointField(name=name,
                               offset=offset,
                               datatype=sensor_msgs.PointField.FLOAT32,
                               count=1) for name, offset in [('x', 0), ('y', 4), ('z', 8)]
    ]

    points = np.array(
          [
            [1.0, -1.0, -1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0]
          ],
          dtype=np.float32
    )

    point_cloud.height = 1 # points.shape[1]
    point_cloud.width = len(points) # points.shape[0]

    point_cloud.is_bigendian = False
    point_cloud.point_step = 12
    point_cloud.row_step = 12 * points.shape[0]
    point_cloud.is_dense = int(np.isfinite(points).all())

    point_cloud.data = np.asarray(points, np.float32).tostring()

    # loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        point_cloud.header.stamp = rospy.Time.now()
        pub.publish(point_cloud)
        rate.sleep()


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node("point_cloud_publisher")
    try:
        point_cloud_publisher()
    except rospy.ROSInterruptException:
        pass
