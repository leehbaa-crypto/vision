#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray
import numpy as np

class PointCloudTransformer(object):
    def __init__(self):
        # Initialize fMc as identity matrix
        self.fMc = np.eye(4)
        self.fMc_initialized = False

        # Subscribers
        self.cloud_sub = rospy.Subscriber(
            '/camera/depth/points',
            PointCloud2,
            self.cloud_callback,
            queue_size=10
        )
        self.fMc_sub = rospy.Subscriber(
            'fMc_matrix',
            Float64MultiArray,
            self.fmc_callback,
            queue_size=10
        )

        # Publisher
        self.pub = rospy.Publisher(
            'transformed_points',
            PointCloud2,
            queue_size=10
        )

    def fmc_callback(self, msg):
        # Update the transformation matrix when a new one arrives
        if len(msg.data) == 16:
            # Reshape data into a 4x4 row-major matrix
            self.fMc = np.array(msg.data).reshape((4, 4))
            self.fMc_initialized = True

        else:
            rospy.logwarn(
                "Received fMc matrix of size %d, expected 16",
                len(msg.data)
            )

    def cloud_callback(self, msg):
    # Skip until we have a valid fMc
        if not self.fMc_initialized:
            rospy.logwarn(
                "No fMc matrix received yet. Skipping point cloud transform."
            )
            return

        # Use incoming header but override frame_id to base_link
        header = msg.header
        header.frame_id = "base_link"  # set reference frame for RViz

        fields = msg.fields

        transformed_points = []
        downsample_rate = 70  # e.g., 1/10 다운샘플링

        # iterate and downsample
        for i, p in enumerate(pc2.read_points(msg, skip_nans=False)):
            if i % downsample_rate != 0:
                continue  # 스킵
            x, y, z = p[0], p[1], p[2]
            # Apply homogeneous transformation
            vec = np.array([x, y, z, 1.0])
            t = self.fMc.dot(vec)
            # Reconstruct point tuple: transformed x,y,z + other fields
            new_point = (t[0], t[1], t[2]) + tuple(p[3:])
            transformed_points.append(new_point)

        # Create and publish new PointCloud2 with base_link frame
        new_msg = pc2.create_cloud(header, fields, transformed_points)
        self.pub.publish(new_msg)


if __name__ == '__main__':
    rospy.init_node('pointcloud_transformer', anonymous=False)
    transformer = PointCloudTransformer()
    rospy.loginfo("PointCloud Transformer node started")
    rospy.spin()
