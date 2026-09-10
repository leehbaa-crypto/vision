#ifndef NRS_MATH_H
#define NRS_MATH_H
#include <string>
#include <cmath>
#include <fstream>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>

// 네임스페이스 사용 (필요에 따라 수정)

using namespace std;

class nrs_math
{
public:
    // 6 자유도 로봇의 순방향 운동학 계산 (입력: 각 관절 각도 배열, 단위: degree)
    Eigen::Matrix4f forwardKinematics(float theta[]);

    // DH 파라미터(alpha, a, d, theta)를 이용하여 개별 변환 행렬 계산
    Eigen::Matrix4f calcTransformationMatrix(float alpha, float a, float d, float theta);

    // degree 단위를 radian 단위로 변환
    float deg2rad(float degree);

    float rad2deg(float rad);
};

#endif // NRS_MATH_H
