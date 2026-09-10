#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import tensorflow as tf


class KeyPointClassifier(object):
    def __init__(
        self,
        model_path='/home/nrs/catkin_ws/src/nrs_hand/scripts/model/keypoint_classifier/keypoint_classifier_0410_ver4.tflite',
        num_threads=1,
    ):
        self.interpreter = tf.lite.Interpreter(model_path=model_path,
                                               num_threads=num_threads)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def __call__(
        self,
        landmark_list,
    ):
        input_details_tensor_index = self.input_details[0]['index']
        output_details_tensor_index = self.output_details[0]['index']

        # 학습 클래스 기입 
        prediction_scores = np.zeros(9) # 학습 클래스 개수 (ex.제스처 개수)
        for landmarks in landmark_list:
            input_data = np.array([landmarks], dtype=np.float32)
            self.interpreter.set_tensor(
            input_details_tensor_index,
            input_data
            )
            self.interpreter.invoke()
            # 결과 가져오기 (확률 값)
            result = self.interpreter.get_tensor(output_details_tensor_index)

            # 확률 값을 누적
            prediction_scores += np.squeeze(result)

        result_index = np.argmax(prediction_scores)

        return result_index
