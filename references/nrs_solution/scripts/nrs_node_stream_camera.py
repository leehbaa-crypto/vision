import pyrealsense2 as rs
import rospy
import numpy as np
import cv2
import threading
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf

# Initialize ROS node
rospy.init_node("stream_realsense_camera_node")
rgb_pub = rospy.Publisher("camera/rgb/image_raw", Image, queue_size=10)
depth_pub = rospy.Publisher("camera/depth/image_raw", Image, queue_size=10)
points_pub = rospy.Publisher("camera/depth/points", PointCloud2, queue_size=10)
br = tf.TransformBroadcaster()
bridge = CvBridge()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline_profile = pipeline.start(config)

depth_sensor = pipeline_profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.visual_preset, rs.rs400_visual_preset.high_accuracy)
depth_sensor.set_option(rs.option.alternate_ir, 0.0)
depth_sensor.set_option(rs.option.confidence_threshold, 1.0)
depth_sensor.set_option(rs.option.digital_gain, 1.0)
depth_sensor.set_option(rs.option.enable_ir_reflectivity, 0.0)
depth_sensor.set_option(rs.option.enable_max_usable_range, 0.0)
depth_sensor.set_option(rs.option.error_polling_enabled, 1.0)
depth_sensor.set_option(rs.option.frames_queue_size, 16.0)
depth_sensor.set_option(rs.option.freefall_detection_enabled, 1.0)
depth_sensor.set_option(rs.option.global_time_enabled, 0.0)
depth_sensor.set_option(rs.option.inter_cam_sync_mode, 0.0)
depth_sensor.set_option(rs.option.invalidation_bypass, 0.0)
depth_sensor.set_option(rs.option.laser_power, 100.0)
depth_sensor.set_option(rs.option.min_distance, 0.0)
depth_sensor.set_option(rs.option.noise_filtering, 6.0)
depth_sensor.set_option(rs.option.post_processing_sharpening, 1.0)
depth_sensor.set_option(rs.option.pre_processing_sharpening, 0.0)
depth_sensor.set_option(rs.option.receiver_gain, 9.0)

align = rs.align(rs.stream.color)
pc = rs.pointcloud()
depth_scale = depth_sensor.get_depth_scale()
rospy.loginfo(f"Depth Scale: {depth_scale}")

# 공유 변수
latest_color_frame = None
latest_depth_frame = None
frame_lock = threading.Lock()

# 별도 스레드에서 PointCloud 발행
def publish_pointcloud_thread():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with frame_lock:
            if latest_color_frame is None or latest_depth_frame is None:
                continue
            color_frame = latest_color_frame
            depth_frame = latest_depth_frame

        # TF broadcast
        now = rospy.Time.now()
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         now,
                         "camera_depth_frame",
                         "map")

        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        vtx = vtx[::4]  # downsample

        header = Header()
        header.stamp = now
        header.frame_id = "camera_depth_frame"
        pc_msg = pc2.create_cloud_xyz32(header, vtx.tolist())
        points_pub.publish(pc_msg)
        rate.sleep()


# PointCloud 쓰레드 시작
threading.Thread(target=publish_pointcloud_thread, daemon=True).start()

# Main loop (30 FPS RGB/Depth publishing)
rate = rospy.Rate(30)
try:
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            rospy.logwarn("Failed to capture aligned frames from RealSense camera.")
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        rgb_pub.publish(color_msg)
        depth_pub.publish(depth_msg)

        # 공유 프레임 업데이트 (PointCloud용)
        with frame_lock:
            latest_color_frame = color_frame
            latest_depth_frame = depth_frame

        rate.sleep()

finally:
    pipeline.stop()
