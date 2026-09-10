#!/usr/bin/env python3
import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped
from datetime import datetime

def lookup_tip(tfbuf, base_frame, tip_frame):
    trans = tfbuf.lookup_transform(base_frame, tip_frame, rospy.Time(0), rospy.Duration(0.5))
    t = trans.transform.translation
    return t.x, t.y, t.z

if __name__ == "__main__":
    rospy.init_node("waypoint_teaching_node")
    base_frame = rospy.get_param("~base_frame", "base_link")
    tip_frame  = rospy.get_param("~tip_frame",  "tool_tip")  # A안이면 tool0, B안이면 tool_tip
    save_path  = rospy.get_param("~save_path",  "/home/nrs/catkin_ws/src/nrs_path/data/selected_waypoints.txt")

    tfbuf = tf2_ros.Buffer()
    tflis = tf2_ros.TransformListener(tfbuf)

    rospy.loginfo(f"base_frame={base_frame}, tip_frame={tip_frame}")
    rospy.loginfo("Enter 칠 때마다 현재 툴팁 좌표 저장. 종료 Ctrl+C")

    # 여기서 파일을 'w' 모드로 열어 초기화합니다.
    with open(save_path, "w") as f:
        # 선택 사항: 파일 시작에 타임스탬프 주석 한 줄 남기고 싶으면 아래 주석 해제
        # f.write(f"# saved_at {datetime.now().isoformat(timespec='seconds')}\n"); f.flush()

        # TF 버퍼가 차도록 아주 짧게 대기
        rospy.sleep(0.3)

        while not rospy.is_shutdown():
            try:
                _ = input()  # 사용자가 바닥 찍은 순간 Enter
                x, y, z = lookup_tip(tfbuf, base_frame, tip_frame)
                line = f"{x:.6f} {y:.6f} {z:.6f}\n"
                f.write(line)
                f.flush()
                print(f"saved: {line.strip()}")
            except Exception as e:
                rospy.logwarn(str(e))
