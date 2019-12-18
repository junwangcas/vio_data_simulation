//
// Created by hyj on 18-1-19.
//

#ifndef IMUSIMWITHPOINTLINE_IMU_H
#define IMUSIMWITHPOINTLINE_IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "param.h"

struct MotionData
{
    /// @brief 运动数据
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// 每一个数据都附带一个时间戳
    double timestamp;
    /// 位置，姿态
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    /// 加速度计与陀螺仪
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;
    /// 加速度计与陀螺仪偏置
    Eigen::Vector3d imu_gyro_bias;
    Eigen::Vector3d imu_acc_bias;
    /// 速度
    Eigen::Vector3d imu_velocity;
};

// euler2Rotation:   body frame to interitail frame
///
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles);
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);


class IMU
{
    ///@brief IMU的相关操作；
public:
    IMU(Param p);
    /// 参数
    Param param_;
    /// 陀螺仪与加速度计的偏置
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d acc_bias_;

    /// 初始速度，初始位置，初始姿态．
    Eigen::Vector3d init_velocity_;
    Eigen::Vector3d init_twb_;
    Eigen::Matrix3d init_Rwb_;

    /// 运动模型
    MotionData MotionModel(double t);

    /// 增加噪声
    void addIMUnoise(MotionData& data);
    void testImu(std::string src, std::string dist);        // imu数据进行积分，用来看imu轨迹
    void testPreintergration(std::string src, std::string dist);             // 使用逐项积分与预积分
};

#endif //IMUSIMWITHPOINTLINE_IMU_H
