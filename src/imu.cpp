//
// Created by hyj on 18-1-19.
//

#include <random>
#include "imu.h"
#include "utilities.h"

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;

}

MotionData IMU::MotionModel(double t)
{
    /// @param t, 绝对时间ｔ，只用作打时间标签．
    MotionData data;
    // param
    /// 这里假设ｘ＝ｙ；
    float ellipse_x = 20;
    float ellipse_y = 20;
    float z = 0;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    /// k*t = 0-2pi, 相当于角度．
    float K = 2*M_PI/ 20;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    /// 一个圆上的位置．
    Eigen::Vector3d position( ellipse_x * cos( K * t), ellipse_y * sin( K * t),  z * sin( K1 * K * t ));
    /// 位置对ｔ求导 = 速度
    Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
    double K2 = K*K;
    ///　位置对ｔ求二阶导　＝　加速度
    Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

    /// 因为是6 DOF的，因此位置与姿态是解耦的，可以随意生成姿态．
    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    /// 姿态对时间的导数，就是角速度
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    /// 由欧拉角转换为旋转矩阵；
    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    ///@todo 当使用欧拉角来进行旋转时，就涉及到角度顺序了．
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

    /// Ｒ＊局部加速度（测量加速度）　＝　全局加速度 - 重力加速度
    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;

}

void IMU::testPreintergration(std::string src, std::string dist) {
    /// @brief 读取前面生成的IMU数据，利用预积分，对整个轨迹进行恢复．

    /// IMU的数据保存在
    std::vector<MotionData> imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;

    /// 对于姿态Ｑ
    Eigen::Quaterniond Qwb0_100(init_Rwb_);
    Eigen::Quaterniond Qwb100_200(init_Rwb_);

    for (int i = 0; i < 100; i++) {
        MotionData imudata_one = imudata[i];
        /// increamental value;
        Eigen::Vector3d dw_half;
        dw_half = imudata_one.imu_gyro*dt*0.5;
        Eigen::Quaterniond dq;
        dq.w() = 1;
        dq.x() = dw_half.x();
        dq.y() = dw_half.y();
        dq.z() = dw_half.z();
        Qwb0_100 = Qwb0_100*dq;
    }

    for (int i = 100; i < 200; i++) {
        MotionData imudata_one = imudata[i];
        /// increamental value;
        Eigen::Vector3d dw_half;
        dw_half = imudata_one.imu_gyro*dt*0.5;
        Eigen::Quaterniond dq;
        dq.w() = 1;
        dq.x() = dw_half.x();
        dq.y() = dw_half.y();
        dq.z() = dw_half.z();
        Qwb100_200 = Qwb100_200*dq;
    }
    /// final pose
    Eigen::Quaterniond Qwbfinal = Qwb0_100*Qwb100_200;
    Eigen::Quaterniond Qwb_data = Eigen::Quaterniond(imudata[199].Rwb);
    Eigen::Quaterniond Qwb_resisual = Qwbfinal.inverse()*Qwb_data;
    std::cout << Qwbfinal.w() << " " << Qwbfinal.x() << " " << Qwbfinal.y() << " " << Qwbfinal.z() << "\n";
    std::cout << Qwb_data.w() << " " << Qwb_data.x() << " " << Qwb_data.y() << " " << Qwb_data.z() << "\n";
    std::cout << Qwb_resisual.w() << " " << Qwb_resisual.x() << " " << Qwb_resisual.y() << " " << Qwb_resisual.z() << "\n";
}

void IMU::testImu(std::string src, std::string dist)
{
    ///@brief //读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
    ////用来验证数据以及模型的有效性。
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    int integral_model = 1; /// 0 欧拉积分，１中值积分；
    for (int i = 1; i < imudata.size(); ++i) {
        MotionData imupose_pre = imudata[i-1];
        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half;
        Eigen::Vector3d acc_w;
        switch (integral_model) {
            case 0:
                /// imu 动力学模型 欧拉积分(我们自己的积分就是用的这种)
                dtheta_half = imupose.imu_gyro * dt / 2.0;
                acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
                break;
            case 1:
                /// 中值积分
                dtheta_half = (imupose.imu_gyro + imupose_pre.imu_gyro)/2.0 * dt / 2.0;
                acc_w = Qwb * ((imupose.imu_acc + imupose_pre.imu_acc)/2.0) + gw;
        }

        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();

        /// 更新姿态　qwb* = qwb*dq;
        /// 更新位置　Pwb* = Pwb + vdt + 0.5 a*t^2;
        /// 更新速度：Vw = Vw + a*dt;

        Qwb = Qwb * dq;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

    }

    std::cout<<"test　end"<<std::endl;

}
