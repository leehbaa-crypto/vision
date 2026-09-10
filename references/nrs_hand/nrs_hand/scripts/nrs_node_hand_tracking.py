import cv2
import mediapipe as mp
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point


# Mediapipe Hand Tracking 모델 초기화
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.6)

# ROS 노드 초기화
rospy.init_node("nrs_node_hand_tracking")
transformation_matrix_pub = rospy.Publisher(
    "hand2cam_matrix", Float64MultiArray, queue_size=10
)
# 추가 Publisher
hand_point_pub = rospy.Publisher("hand_pointed_position", 
                                 Point, queue_size=10)


# CvBridge 초기화
bridge = CvBridge()

# Depth 스케일 및 Intrinsic Parameters 설정
depth_scale = 0.000250
color_fx = 599.0752563  # Color Focal length x
color_fy = 599.5353394   # Color Focal length y
color_cx = 325.3421021  # Color Optical center x
color_cy = 242.0690002  # Color Optical center y

depth_fx = 461.7890625       # Depth Focal length x
depth_fy = 461.4375  # Depth Focal length y
depth_cx = 304.7851562  # Depth Optical center x
depth_cy = 253.7578125  # Depth Optical center y

# color_fx = 300.0752563  # Color Focal length x
# color_fy = 200.5353394   # Color Focal length y
# color_cx = 500.3421021  # Color Optical center x
# color_cy = 700.0690002  # Color Optical center y

# depth_fx = 400.7890625       # Depth Focal length x
# depth_fy = 800.4375  # Depth Focal length y
# depth_cx = 254.7851562  # Depth Optical center x
# depth_cy = 203.7578125  # Depth Optical center y


kud,kdu = -0.1600932628, 0.1600932628
# 전역 변수
rgb_image = None
depth_image = None
fMc_matrix = np.eye(4, dtype=np.float64)  # 기본값으로 단위 행렬

def pixel_to_camera(u, v, depth, fx, fy, cx, cy, kud, kdu):
    """Convert pixel coordinates to camera coordinates with distortion"""
    # Calculate undistorted depth
    Z = depth * depth_scale
    if Z == 0:
        return None

    # Normalize pixel coordinates
    x = (u - cx) / fx
    y = (v - cy) / fy

    # Apply inverse distortion model
    r2 = x * x + y * y  # Radial distance squared
    distortion_factor = 1 + kud * r2 + kdu * (r2 ** 2)

    x_undistorted = x / distortion_factor
    y_undistorted = y / distortion_factor

    # Calculate camera coordinates
    X = x_undistorted * Z
    Y = y_undistorted * Z

    return np.array([X, Y, Z])

def calculate_pose(points_3d, fMc_matrix, option):
    avg_position = points_3d[1]

    if option == 0:  # default
        v1 = points_3d[1] - points_3d[0]
        v2 = points_3d[2] - points_3d[0]
        z_axis = np.cross(v2, v1)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = points_3d[1] - points_3d[2]
        x_axis /= np.linalg.norm(x_axis)

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = avg_position

    elif option == 1:  # 평행하게 움직이는 모드

        # Fixed z_axis to [0, 0, -1]
        z_axis = np.array([0, 0, -1])

        # Compute x_axis based on the first two points
        x_axis = points_3d[1] - points_3d[2]
        x_axis /= np.linalg.norm(x_axis)

        # Compute y_axis as the cross product of z_axis and x_axis
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = avg_position
    
    elif option == 2:  # 평행, 고정된 방향으로 움직이는 모드

        # Fixed z_axis to [0, 0, -1]
        z_axis = np.array([0, 0, -1])

        # Compute x_axis based on the first two points
        x_axis = np.array([1,0,0], dtype=float)
        x_axis /= np.linalg.norm(x_axis)

        # Compute y_axis as the cross product of z_axis and x_axis
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = avg_position
    
    elif option == 3:  # 30도 각도 안정화
        angle_threshold = np.deg2rad(30)  # 30도 (라디안 변환)
        
        default_axis=np.array([[-1,1,0],[-1,-1,0],[0,0,1]])

        # 현재 축 계산
        v1 = points_3d[1] - points_3d[0]
        v2 = points_3d[2] - points_3d[0]
        # z_axis
        z_axis = np.cross(v2, v1)
        z_norm = np.linalg.norm(z_axis)
        if z_norm == 0:
            rospy.logwarn("Degenerate points for z_axis. Skipping pose calculation.")
            return None, None, None, None, None
        z_axis /= z_norm

        # x_axis
        x_axis = points_3d[1] - points_3d[2]
        x_norm = np.linalg.norm(x_axis)
        if x_norm == 0:
            rospy.logwarn("Degenerate points for x_axis. Skipping pose calculation.")
            return None, None, None, None, None
        x_axis /= x_norm

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        if z_axis[2] > 0:  # z축 방향이 위를 향하면
            z_axis = -z_axis  # 아래로 향하도록 반전

        # 회전 행렬 및 변환 행렬 생성
        rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = avg_position

        # fMc_matrix 적용
        transformed_matrix = np.dot(fMc_matrix, transformation_matrix)
        transformed_rotation = transformed_matrix[:3, :3]  # 회전 부분

        def compute_rotation_difference(R1, R2):

            assert R1.shape == (3, 3) and R2.shape == (3, 3), "입력 행렬은 3x3이어야 합니다."

            # R1과 R2의 상대 회전 행렬 계산 (R_relative = R1^T * R2)
            R_relative = np.dot(R1.T, R2)

            # SciPy를 이용하여 오일러 각도 변환
            euler_angles = R.from_matrix(R_relative).as_euler('xyz', degrees=False)

            return euler_angles  # (x_diff, y_diff, z_diff)

        x_angle, y_angle, z_angle = compute_rotation_difference(default_axis, transformed_rotation)

        # 축이 45도 이하로 차이 나는 경우
        if np.abs(z_angle) <= angle_threshold:
            Rz = np.array([
            [np.cos(z_angle), -np.sin(z_angle), 0],
            [np.sin(z_angle),  np.cos(z_angle), 0],
            [0, 0, 1]])
            corrected_rotation = np.dot(Rz, transformation_matrix[:3, :3])
            transformation_matrix[:3, :3] = corrected_rotation

        if np.abs(y_angle) <= angle_threshold:
            Ry = np.array([
            [np.cos(y_angle), 0, np.sin(y_angle)],
            [0, 1, 0],
            [-np.sin(y_angle), 0, np.cos(y_angle)]
            ])
            corrected_rotation = np.dot(Ry, transformation_matrix[:3, :3])
            transformation_matrix[:3, :3] = corrected_rotation

        if np.abs(x_angle) <= angle_threshold:
            Rx = np.array([
            [1, 0, 0],
            [0, np.cos(-x_angle), -np.sin(-x_angle)],
            [0, np.sin(-x_angle), np.cos(-x_angle)]
            ])
            corrected_rotation = np.dot(Rx, transformation_matrix[:3, :3])
            transformation_matrix[:3, :3] = corrected_rotation

    return avg_position, x_axis, y_axis, z_axis, transformation_matrix

def draw_axes_on_image(image, origin_cam, x_axis, y_axis, z_axis, 
                       fx, fy, cx, cy, scale=0.1, thickness=2):
    """
    image: 그릴 대상 (BGR) 이미지
    origin_cam: 카메라 좌표계 [X,Y,Z] (meters)
    x_axis, y_axis, z_axis: 카메라 좌표계 기준 단위벡터(길이 1)
    fx, fy, cx, cy: 투영에 사용할 카메라 내참수 (RGB용)
    scale: 축 길이 (meters)
    """
    def proj(P):
        X, Y, Z = P
        if Z <= 0:
            return None
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy
        return (int(round(u)), int(round(v)))

    # 원점과 각 축 끝점
    p0 = proj(origin_cam)
    px = proj(origin_cam + x_axis * scale)
    py = proj(origin_cam + y_axis * scale)
    pz = proj(origin_cam + z_axis * scale)

    if p0 is None:
        return  # Z<=0이면 안 그림

    # X(빨강), Y(초록), Z(파랑)
    if px is not None:
        cv2.line(image, p0, px, (0, 0, 255), thickness)
    if py is not None:
        cv2.line(image, p0, py, (0, 255, 0), thickness)
    if pz is not None:
        cv2.line(image, p0, pz, (255, 0, 0), thickness)


def expand_hand_region(depth_image_raw, hand_landmarks, image_shape, expansion_radius_px=7):
    """
    landmark 기반 손 마스크를 생성하고, 그 영역만 팽창하여 depth 보정.
    """
    # 1. 랜드마크 기반 손 마스크 생성
    hand_mask = create_hand_mask(image_shape, hand_landmarks)

    # 2. 팽창 (dilate)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (2 * expansion_radius_px + 1,
                                        2 * expansion_radius_px + 1))
    dilated_mask = cv2.dilate(hand_mask, kernel)

    # 4. erosion에 대비한 클린 depth 이미지 (0값 → 큰 값으로 치환)
    depth_clean = depth_image_raw.copy()
    depth_clean[(depth_clean == 0)] = 10000

    # 5. 침식 → 주변 depth의 최소값 확보
    eroded_depth = cv2.erode(depth_clean, kernel)

    # 6. 확장된 주변만 최소값으로 보정
    expanded_depth = np.where(dilated_mask == 0, depth_image_raw, eroded_depth)
    

    return expanded_depth

def create_hand_mask(image_shape, hand_landmarks, circle_radius_px=20):
    """
    손가락 뼈대를 기반으로 빠르게 마스크 생성 (최적화)
    """
    mask = np.zeros(image_shape[:2], dtype=np.uint8)
    img_h, img_w = image_shape[:2]

    # 좌표 변환 한 번에
    lm_xy = np.array([
        [int(lm.x * img_w), int(lm.y * img_h)]
        for lm in hand_landmarks.landmark
    ])

    # 손가락 라인에 원 그리기만 (폴리곤 생략 가능)
    for x, y in lm_xy:
        cv2.circle(mask, (x, y), circle_radius_px, 255, -1)

    return mask 


def process_frame(rgb_image, depth_image):
    global fMc_matrix

    # 1) 프레임 전처리
    rgb_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    # 👉 항상 depth_colormap 기본 생성
    depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03)
    depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    filtered_colormap = depth_colormap.copy()  # 기본은 동일하게 복사

    hand_mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)

    # 2) 손이 감지되었을 때만 로직 실행
    if results.multi_hand_landmarks and results.multi_handedness:
        for handedness, hand_landmarks in zip(results.multi_handedness,
                                              results.multi_hand_landmarks):
            label = handedness.classification[0].label
            if label != 'Left':
                continue

            # hand_mask = create_hand_mask(rgb_image.shape, hand_landmarks)
            filtered_depth = expand_hand_region(depth_image, hand_landmarks, rgb_image.shape)

            # # filtered_depth → 컬러맵
            # filtered_vis = cv2.convertScaleAbs(filtered_depth, alpha=0.03)
            # filtered_colormap = cv2.applyColorMap(filtered_vis, cv2.COLORMAP_JET)

            # --- 오른손에 대해서만 3점으로 pose 계산 ---   
            indices = [0, 5, 9]
            points_3d = []
            for idx in indices:
                lm = hand_landmarks.landmark[idx]
                px = int(lm.x * rgb_image.shape[1])
                py = int(lm.y * rgb_image.shape[0])
                if 0 <= px < depth_image.shape[1] and 0 <= py < depth_image.shape[0]:
                    d = depth_image[py, px]
                    pt = pixel_to_camera(px, py, d,
                                         color_fx, color_fy, color_cx, color_cy,
                                         kud, kdu)
                    if pt is not None:
                        points_3d.append(pt)
                        # cv2.circle(rgb_image, (px, py), 5, (0, 255, 0), -1)

            # --- 검지 8번 랜드마크 publish ---
            lm8 = hand_landmarks.landmark[8]
            p8x = int(lm8.x * rgb_image.shape[1])
            p8y = int(lm8.y * rgb_image.shape[0])
            if 0 <= p8x < filtered_depth.shape[1] and 0 <= p8y < filtered_depth.shape[0]:
                d8 = filtered_depth[p8y, p8x]
                pt8 = pixel_to_camera(p8x, p8y, d8,
                                      color_fx, color_fy, color_cx, color_cy,
                                      kud, kdu)
                if pt8 is not None:
                    tp8 = np.dot(fMc_matrix, np.append(pt8, 1))[:3]
                    hand_point_pub.publish(Point(*tp8))
                    # cv2.circle(rgb_image, (p8x, p8y), 5, (255, 255, 0), -1)

                    depth_meters = d8 * depth_scale
                    text = f"{depth_meters:.3f}m"

                    # cv2.putText(rgb_image, text, (p8x + 10, p8y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    # cv2.putText(depth_colormap, text, (p8x + 10, p8y - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    # cv2.putText(filtered_colormap, text, (p8x + 10, p8y - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # --- pose 계산 & 행렬 publish ---
            if len(points_3d) == 3:
                avg_pos, x_ax, y_ax, z_ax, tfm = calculate_pose(points_3d, fMc_matrix, option=3)
                if tfm is not None:
                    transformation_matrix_pub.publish(Float64MultiArray(data=tfm.flatten().tolist()))
        #             draw_axes_on_image(
        #     rgb_image,           # 그릴 대상
        #     avg_pos,             # 카메라좌표 원점(손등 위치)
        #     x_ax, y_ax, z_ax,    # 축 벡터(단위)
        #     color_fx, color_fy, color_cx, color_cy,  # RGB intrinsics
        #     scale=0.10,          # 10 cm 길이 축
        #     thickness=2
        # )


    
    # hand_mask_colored = cv2.cvtColor(hand_mask, cv2.COLOR_GRAY2BGR)
    # hand_overlay = cv2.addWeighted(rgb_image, 1.0, hand_mask_colored, 0.5, 0)

    
    # combined = np.hstack((depth_colormap, filtered_colormap))
    # cv2.imshow("Depth Original | Filtered", combined)
    # cv2.imshow("Hand Mask Overlay", hand_overlay)
    cv2.imshow("Hand Tracking with Mediapipe", rgb_image)
    cv2.waitKey(1)

def rgb_callback(data):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

def depth_callback(data):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(data, "16UC1")

def fMc_callback(data):
    global fMc_matrix
    if len(data.data) == 16:
        fMc_matrix = np.array(data.data).reshape(4, 4)

# ROS Subscribers
rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback)
rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)
rospy.Subscriber("fMc_matrix", Float64MultiArray, fMc_callback)

# Main loop
rate = rospy.Rate(60)
while not rospy.is_shutdown():
    if rgb_image is not None and depth_image is not None:
        process_frame(rgb_image, depth_image)
    rate.sleep()

cv2.destroyAllWindows()