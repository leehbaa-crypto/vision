
import rospy
from geometry_msgs.msg import Point

class HandPointRecorder:
    def __init__(self, filename):
        rospy.init_node('hand_point_recorder', anonymous=True)
        self.file = open(filename, 'w')
        rospy.Subscriber('hand_pointed_position', Point, self.callback)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(f"Recording hand_pointed_position → {filename}")
        rospy.spin()

    def callback(self, msg: Point):
        # 한 줄에 x y z 형식으로 저장
        line = f"{msg.x:.6f} {msg.y:.6f} {msg.z:.6f}\n"
        self.file.write(line)
        # 바로 디스크에 써두고 싶다면
        self.file.flush()

    def on_shutdown(self):
        rospy.loginfo("Shutting down, closing file.")
        self.file.close()

if __name__ == '__main__':
    # 원하는 파일 경로로 수정하세요
    recorder = HandPointRecorder('data/hand_pointed_position.txt')
