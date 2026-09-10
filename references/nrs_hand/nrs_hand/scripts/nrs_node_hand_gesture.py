#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import csv
import copy
import itertools
import math
from collections import deque
import cv2 as cv
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os
sys.path.append(os.path.dirname(__file__))
from model.keypoint_classifier.keypoint_classifier import KeyPointClassifier
from model.point_history_classifier.point_history_classifier import PointHistoryClassifier

rgb_image = None
bridge = CvBridge()

def rgb_callback(data):
    global rgb_image
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")


def rotate_point(x, y, angle):
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a

def generate_rotated_landmarks(landmark_list):
    rotated_landmarks = []
    for angle in range(0, 360, 60):
        rotated = []
        for x, y in landmark_list:
            rotated.append(rotate_point(x, y, angle))
        rotated_landmarks.append(rotated)
    return rotated_landmarks

def pre_process_landmark(landmark_list):
    temp = copy.deepcopy(landmark_list)

    base_x, base_y = temp[0]
    for pt in temp:
        pt[0] -= base_x
        pt[1] -= base_y

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
        pts.append([min(int(lm.x * w), w - 1), min(int(lm.y * h), h - 1)])
    return pts

def main():
    rospy.init_node('nrs_node_hand_gesture')
    pub = rospy.Publisher('/hand_gesture', String, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz

    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback)

    # MediaPipe Hands
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

    try:
        while not rospy.is_shutdown():
            if rgb_image is None:
                rate.sleep()
                continue

            image = cv.flip(rgb_image.copy(), 1)
            rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            rgb.flags.writeable = True

            left_label = ''
            right_label = ''

            if results.multi_hand_landmarks:
                for lm, handedness in zip(results.multi_hand_landmarks,
                                           results.multi_handedness):
                    lm_list = calc_landmark_list(image, lm)
                    proc_ln = pre_process_landmark(lm_list)

                    sign_id = keypoint_classifier(proc_ln)
                    sign_label = keypoint_classifier_labels[sign_id]

                    hand_label = handedness.classification[0].label
                    if hand_label == 'Left':
                        left_label = sign_label
                    else:
                        right_label = sign_label

            out_str = f"Left:{left_label},Right:{right_label}"
            pub.publish(out_str)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
