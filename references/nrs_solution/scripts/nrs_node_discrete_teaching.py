#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import os

class DiscreteTeachingNode:
    def __init__(self):
        rospy.init_node("nrs_node_discrete_teaching")

        self.latest_point = None
        self.output_path = "/home/nrs/catkin_ws/src/nrs_path/data/selected_waypoints.txt"

        # 파일 초기화
        with open(self.output_path, "w") as f:
            f.write("")
        rospy.loginfo(f"[DiscreteTeaching] File initialized: {self.output_path}")

        rospy.Subscriber("hand_pointed_position", Point, self.point_callback)

        # 서비스 서버 등록
        rospy.Service("/discrete_teaching_save", Empty, self.save_callback)
        rospy.Service("/discrete_teaching_end", Empty, self.end_callback)

    def point_callback(self, msg):
        self.latest_point = np.array([msg.x, msg.y, msg.z])

    def save_callback(self, req):
        if self.latest_point is not None:
            with open(self.output_path, "a") as f:
                line = f"{self.latest_point[0]:.6f} {self.latest_point[1]:.6f} {self.latest_point[2]+0.03:.6f}\n"
                f.write(line)
            rospy.loginfo(f"[DiscreteTeaching] Saved waypoint: {line.strip()}")
        else:
            rospy.logwarn("[DiscreteTeaching] No valid point to save.")
        return EmptyResponse()

    def end_callback(self, req):
        if not os.path.exists(self.output_path):
            rospy.logwarn("[DiscreteTeaching] File not found on end.")
            return EmptyResponse()

        with open(self.output_path, "r") as f:
            lines = f.readlines()

        with open(self.output_path, "w") as f:
            f.writelines(lines)

        rospy.loginfo("[DiscreteTeaching] Teaching session ended.")
        return EmptyResponse()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DiscreteTeachingNode()
    node.run()
