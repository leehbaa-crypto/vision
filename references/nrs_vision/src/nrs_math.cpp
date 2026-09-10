#include "nrs_math.h"

Eigen::Matrix4f nrs_math::forwardKinematics(float theta[])
{
    // UR10e 파라미터 (단위: meter, radian)
    float alpha[6] = {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0.0};
    float a[6] = {0.0, -0.612, -0.5723, 0.0, 0.0, 0.0};
    float d[6] = {0.1273, 0.0, 0.0, 0.163941, 0.1157, 0.0922};

    // 입력된 각도를 degree 단위에서 radian 단위로 변환
    for (int i = 0; i < 6; ++i)
    {
        theta[i] = deg2rad(theta[i]);
    }

    Eigen::Matrix4f T01 = calcTransformationMatrix(alpha[0], a[0], d[0], theta[0]);
    Eigen::Matrix4f T12 = calcTransformationMatrix(alpha[1], a[1], d[1], theta[1]);
    Eigen::Matrix4f T23 = calcTransformationMatrix(alpha[2], a[2], d[2], theta[2]);
    Eigen::Matrix4f T34 = calcTransformationMatrix(alpha[3], a[3], d[3], theta[3]);
    Eigen::Matrix4f T45 = calcTransformationMatrix(alpha[4], a[4], d[4], theta[4]);
    Eigen::Matrix4f T56 = calcTransformationMatrix(alpha[5], a[5], d[5], theta[5]);

    return T01 * T12 * T23 * T34 * T45 * T56;
}

Eigen::Matrix4f nrs_math::calcTransformationMatrix(float alpha, float a, float d, float theta)
{
    Eigen::Matrix4f individualTransformationMatrix;
    individualTransformationMatrix << cos(theta), (-sin(theta) * cos(alpha)), (sin(theta) * sin(alpha)), (a * cos(theta)),
        sin(theta), (cos(theta) * cos(alpha)), (-cos(theta) * sin(alpha)), (a * sin(theta)),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;
    return individualTransformationMatrix;
}

float nrs_math::deg2rad(float degree)
{
    return degree * M_PI / 180.0;
}

float nrs_math::rad2deg(float rad)
{
    return rad * (180.0f / M_PI);
}
