#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import copy
import itertools
import math
import cv2 as cv
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import sys
import os
sys.path.append(os.path.dirname(__file__))
from model.keypoint_classifier.keypoint_classifier import KeyPointClassifier
from model.point_history_classifier.point_history_classifier import PointHistoryClassifier

rgb_image = None
bridge = CvBridge()
FONT = cv.FONT_HERSHEY_SIMPLEX

def rgb_callback(msg):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def rotate_point(x, y, angle_deg):
    r = math.radians(angle_deg)
    c, s = math.cos(r), math.sin(r)
    return x * c - y * s, x * s + y * c

def generate_rotated_landmarks(landmark_list):
    out = []
    for angle in range(0, 360, 60):
        out.append([list(rotate_point(x, y, angle)) for x, y in landmark_list])
    return out

def pre_process_landmark(landmark_list):
    temp = copy.deepcopy(landmark_list)
    base_x, base_y = temp[0]
    for p in temp:
        p[0] -= base_x
        p[1] -= base_y
    processed = []
    for rotated in generate_rotated_landmarks(temp):
        flat = list(itertools.chain.from_iterable(rotated))
        max_v = max(map(abs, flat)) if flat else 1
        processed.append(np.array([v / max_v for v in flat], dtype=np.float32))
    return processed

def calc_landmark_list(image, landmarks):
    h, w = image.shape[:2]
    pts = []
    for lm in landmarks.landmark:
        x = min(int(lm.x * w), w - 1)
        y = min(int(lm.y * h), h - 1)
        pts.append([x, y])
    return pts

def draw_hand_label(image, hand_landmarks, handedness, gesture_label):
    h, w = image.shape[:2]
    x0 = int(hand_landmarks.landmark[0].x * w)
    y0 = int(hand_landmarks.landmark[0].y * h)
    text = f"{handedness}: {gesture_label}" if gesture_label else handedness
    (tw, th), _ = cv.getTextSize(text, FONT, 0.7, 2)
    cv.rectangle(image, (x0, y0 - th - 8), (x0 + tw + 6, y0 + 4), (0, 0, 0), -1)
    cv.putText(image, text, (x0 + 3, y0 - 5), FONT, 0.7, (255, 255, 255), 2, cv.LINE_AA)

def draw_fps(image, fps):
    s = f"FPS: {fps:.1f}"
    cv.putText(image, s, (10, 30), FONT, 0.8, (0, 0, 0), 3, cv.LINE_AA)
    cv.putText(image, s, (10, 30), FONT, 0.8, (255, 255, 255), 1, cv.LINE_AA)

def main():
    rospy.init_node('nrs_node_hand_gesture_visual_only')
    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback)
    rate = rospy.Rate(30)

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False,
                           max_num_hands=2,
                           min_detection_confidence=0.8,
                           min_tracking_confidence=0.5)

    keypoint_classifier = KeyPointClassifier()
    point_history_classifier = PointHistoryClassifier()

    with open('/home/nrs/catkin_ws/src/nrs_hand/scripts/model/keypoint_classifier/keypoint_classifier_label.csv',
              encoding='utf-8-sig') as f:
        keypoint_classifier_labels = [row[0] for row in csv.reader(f)]

    prev_t = time.time()
    fps = 0.0

    try:
        while not rospy.is_shutdown():
            if rgb_image is None:
                rate.sleep()
                continue

            img = cv.flip(rgb_image.copy(), 1)
            rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            rgb.flags.writeable = True

            if results.multi_hand_landmarks and results.multi_handedness:
                for lm, handedness in zip(results.multi_hand_landmarks,
                                          results.multi_handedness):
                    lm_list = calc_landmark_list(img, lm)
                    proc_ln = pre_process_landmark(lm_list)
                    sign_id = keypoint_classifier(proc_ln)
                    gesture = keypoint_classifier_labels[sign_id]
                    hand_label = handedness.classification[0].label
                    # 랜드마크 그리기 없이 라벨만 표시
                    draw_hand_label(img, lm, hand_label, gesture)

            now = time.time()
            dt = now - prev_t
            prev_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else (1.0 / dt)
            draw_fps(img, fps)

            cv.imshow("Hand Gesture Visualization", img)
            k = cv.waitKey(1) & 0xFF
            if k == 27 or k == ord('q'):
                break

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()
