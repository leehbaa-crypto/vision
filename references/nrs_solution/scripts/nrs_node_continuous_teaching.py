import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
import numpy as np



import numpy as np
import numpy as np
import rospy


class ContinuousTeachingNode:
    def __init__(self):
        rospy.init_node("nrs_node_continuous_teaching")

        self.save_hz = rospy.get_param("~save_hz", 30.0)
        self.save_interval = rospy.Duration(1.0 / self.save_hz)
        self.last_save_time = rospy.Time(0)

        self.latest_point = None
        self.last_saved_point = None
        self.file_closed = False

        rospy.Subscriber("hand_pointed_position", Point, self.point_callback, queue_size=1)

        self.output_path = "/home/nrs/catkin_ws/src/nrs_path/data/continuous_teaching_waypoints.txt"
        self.file = open(self.output_path, "w")
        rospy.loginfo(f"[ContinuousTeaching] Writing waypoints to {self.output_path}")

        rospy.Service("/continuous_teaching_save", Empty, self.handle_save)
        rospy.Service("/continuous_teaching_end", Empty, self.handle_end)

    def point_callback(self, msg):

        fx, fy, fz = msg.x, msg.y, msg.z
        self.latest_point = np.array([fx, fy, fz])

    def handle_save(self, req):
        now = rospy.Time.now()

        if now - self.last_save_time < self.save_interval:
            rospy.logdebug("[ContinuousTeaching] Skipping save: interval not elapsed")
            return EmptyResponse()

        if self.latest_point is None:
            rospy.logwarn("[ContinuousTeaching] No point to save.")
            return EmptyResponse()

        if self.last_saved_point is not None and np.allclose(self.latest_point, self.last_saved_point, atol=1e-6):
            rospy.logdebug(f"[ContinuousTeaching] Skipping duplicate point: {self.latest_point}")
            return EmptyResponse()

        line = f"{self.latest_point[0]:.6f} {self.latest_point[1]:.6f} {self.latest_point[2]+0.03:.6f}\n"
        self.file.write(line)
        self.file.flush()
        rospy.loginfo(f"[ContinuousTeaching] Saved waypoint: {line.strip()}")

        self.last_saved_point = self.latest_point.copy()
        self.last_save_time = now
        return EmptyResponse()

    def handle_end(self, req):
        if not self.file_closed:
            self.file.close()
            self.file_closed = True
            rospy.loginfo("[ContinuousTeaching] Closed file.")
        return EmptyResponse()

    def run(self):
        rospy.spin()
        if not self.file_closed:
            self.file.close()
            rospy.loginfo("[ContinuousTeaching] File closed on shutdown.")

if __name__ == '__main__':
    node = ContinuousTeachingNode()
    node.run()
