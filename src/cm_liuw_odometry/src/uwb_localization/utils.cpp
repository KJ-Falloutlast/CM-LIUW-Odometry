#include "uwb_process/utils.h"
#include <yaml-cpp/yaml.h>

// define global variables
Eigen::Vector3d Utils::gravity = Eigen::Vector3d(0, 0, 9.81);

Eigen::Matrix3d Utils::skew(const Eigen::Vector3d &v)
{
     Eigen::Matrix3d w;
     w << 0., -v(2), v(1),
         v(2), 0., -v(0),
         -v(1), v(0), 0.;
     return w;
}

Eigen::Matrix3d Utils::normalizeR(const Eigen::Matrix3d &R)
{
     Eigen::Quaterniond q(R);
     q.normalize();
     return q.toRotationMatrix();
}

Eigen::Vector3d Utils::RToso3(const Eigen::Matrix3d &R_in)
{
     Eigen::Matrix3d R = normalizeR(R_in);
     double theta = acos((R(0, 0) + R(1, 1) + R(2, 2) - 1.0) / 2);
     if (theta < THETA_THRESHOLD)
     {
          return Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) / 2.0;
     }
     else
     {
          return theta * Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) / (2.0 * sin(theta));
     }
}

Eigen::Quaterniond Utils::RToq(const Eigen::Matrix3d &R)
{
     return Eigen::Quaterniond(R);
}

Eigen::Matrix3d Utils::qToR(const Eigen::Quaterniond &q)
{
     return q.toRotationMatrix();
}

Eigen::Matrix3d Utils::so3ToR(const Eigen::Vector3d &so3)
{
     double theta = so3.norm();
     if (theta < THETA_THRESHOLD)
     {
          Eigen::Matrix3d u_x = skew(so3);
          return Eigen::Matrix3d::Identity() + u_x + 0.5 * u_x * u_x;
     }
     else
     {
          Eigen::Matrix3d u_x = skew(so3.normalized());
          return Eigen::Matrix3d::Identity() + sin(theta) * u_x + (1 - cos(theta)) * u_x * u_x;
     }
}

Eigen::Quaterniond Utils::so3Toq(const Eigen::Vector3d &so3)
{
     double theta = so3.norm();
     Eigen::Vector3d u = so3.normalized();
     return Eigen::Quaterniond(cos(0.5 * theta), u.x() * sin(0.5 * theta), u.y() * sin(0.5 * theta), u.z() * sin(0.5 * theta));
}

Eigen::Vector3d Utils::qToso3(const Eigen::Quaterniond &q)
{
     return RToso3(q.toRotationMatrix());
}

//so3 -> eulerAngle
/**
 * def matrix2angle(R):
    ''' get three Euler angles from Rotation Matrix
    Args:
        R: (3,3). rotation matrix
    Returns:
        x: pitch
        y: yaw
        z: roll
    '''
    assert(isRotationMatrix)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 */
Eigen::Vector3d Utils::SO3ToEuler(const SO3d& rot)
{
     // Eigen::Matrix3d mat(rot);//将SO3转换为R，R转欧拉角
     // double sy = sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0));
     // bool singular = sy < 1e-6;
     // double x, y, z;
     // if (!singular)
     // {
     //      x = atan2(mat(2, 1), mat(2, 2));
     //      y = atan2(-mat(2, 0), sy);
     //      z = atan2(mat(1, 0), mat(0, 0));
     // }
     // else
     // {
     //      x = atan2(-mat(1, 2), mat(1, 1));
     //      y = atan2(-mat(2, 0), sy);
     //      z = 0;
     // }
     // Eigen::Vector3d ang(x, y, z);//rad形式

    // 使用Sophus定义一个SO3d对象，假设这是你已经有的旋转

    // 从SO3d对象中获取欧拉角
     Eigen::Vector3d euler_angles = rot.unit_quaternion().toRotationMatrix().eulerAngles(2, 1, 0);
     double x, y, z;
     x = euler_angles[0];//r
     y = euler_angles[1];//p
     z = euler_angles[2];//y
     Eigen::Vector3d ang(x, y, z);//rad形式
     return ang;
}


//Jl
Eigen::Matrix3d Utils::Jl_so3(const Eigen::Vector3d &so3)
{
     double theta = so3.norm();
     if (theta < THETA_THRESHOLD)
     {
          return Eigen::Matrix3d::Identity() + skew(so3) / 2;
     }
     else
     {
          Eigen::Vector3d u = so3.normalized();
          return sin(theta) / theta * Eigen::Matrix3d::Identity() + (1 - sin(theta) / theta) * u * u.transpose() + (1 - cos(theta)) / theta * skew(u);
     }
}

//Jl_inv
Eigen::Matrix3d Utils::Jl_so3_inv(const Eigen::Vector3d &so3)
{
     double theta = so3.norm();
     if (theta < THETA_THRESHOLD)
     {
          return cos(theta / 2) * Eigen::Matrix3d::Identity() + 0.125 * so3 * so3.transpose() - 0.5 * skew(so3);
     }
     else
     {
          Eigen::Vector3d u = so3.normalized();
          return 0.5 * theta / tan(theta / 2) * Eigen::Matrix3d::Identity() + (1 - 0.5 * theta / tan(theta / 2)) * u * u.transpose() - 0.5 * skew(so3);
     }
}

//Jr
Eigen::Matrix3d Utils::Jr_so3(const Eigen::Vector3d &so3)
{
     double theta = so3.norm();
     if (theta < THETA_THRESHOLD)
     {
          return Eigen::Matrix3d::Identity() - 0.5 * skew(so3);
     }
     else
     {
          Eigen::Vector3d u = so3.normalized();
          return sin(theta) / theta * Eigen::Matrix3d::Identity() + (1 - sin(theta) / theta) * u * u.transpose() - (1 - cos(theta)) / theta * skew(u);
     }
}

//Jr_inv
Eigen::Matrix3d Utils::Jr_so3_inv(const Eigen::Vector3d &so3)
{
     double theta = so3.norm();
     if (theta < THETA_THRESHOLD)
     {
          return cos(theta / 2) * Eigen::Matrix3d::Identity() + 0.125 * so3 * so3.transpose() + 0.5 * skew(so3);
     }
     else
     {
          Eigen::Vector3d u = so3.normalized();
          return 0.5 * theta / tan(theta / 2) * Eigen::Matrix3d::Identity() + (1 - 0.5 * theta / tan(theta / 2)) * u * u.transpose() + 0.5 * skew(so3);
     }
}

Eigen::Matrix3d Utils::Jr_R_inv(const Eigen::Matrix3d &R)
{
     Eigen::Vector3d so3 = RToso3(R);
     return Jr_so3_inv(so3);
}

Eigen::Matrix4d Utils::quatLeftMatrix(const Eigen::Quaterniond &q)
{
     Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
     m4.block<3, 1>(1, 0) = q.vec();
     m4.block<1, 3>(0, 1) = -q.vec();
     m4.block<3, 3>(1, 1) = skew(q.vec());
     m4 += Eigen::Matrix4d::Identity() * q.w();
     return m4;
}

Eigen::Matrix4d Utils::quatRightMatrix(const Eigen::Quaterniond &q)
{
     Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
     m4.block<3, 1>(1, 0) = q.vec();
     m4.block<1, 3>(0, 1) = -q.vec();
     m4.block<3, 3>(1, 1) = -skew(q.vec());
     m4 += Eigen::Matrix4d::Identity() * q.w();
     return m4;
}

Eigen::Quaterniond Utils::deltaQ(const Eigen::Vector3d &theta)
{
     Eigen::Quaterniond dq;
     Eigen::Vector3d half_theta = theta;
     half_theta /= 2.0;
     dq.w() = 1.0; // small value can take as 1
     dq.x() = half_theta.x();
     dq.y() = half_theta.y();
     dq.z() = half_theta.z();
     return dq;
}

// Eigen::Isometry3d Utils::odomToEigen(const nav_msgs::OdometryConstPtr &odom_msg)
// {
//      Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
//      Eigen::Quaterniond q_temp(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
//      odom.linear() = q_temp.toRotationMatrix();
//      odom.translation().x() = odom_msg->pose.pose.position.x;
//      odom.translation().y() = odom_msg->pose.pose.position.y;
//      odom.translation().z() = odom_msg->pose.pose.position.z;
//      return odom;
// }

Eigen::Isometry3d Utils::so3ToT(const Eigen::Vector3d &rotation, const Eigen::Vector3d &translation)
{
     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
     T.linear() = so3ToR(rotation);
     T.translation() = translation;
     return T;
}

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
     Eigen::Vector3d n = R.col(0);
     Eigen::Vector3d o = R.col(1);
     Eigen::Vector3d a = R.col(2);

     Eigen::Vector3d ypr(3);
     double y = atan2(n(1), n(0));
     double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
     double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
     ypr(0) = y;
     ypr(1) = p;
     ypr(2) = r;

     return ypr / M_PI * 180.0;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
     typedef typename Derived::Scalar Scalar_t;

     Scalar_t y = ypr(0) / 180.0 * M_PI;
     Scalar_t p = ypr(1) / 180.0 * M_PI;
     Scalar_t r = ypr(2) / 180.0 * M_PI;

     Eigen::Matrix<Scalar_t, 3, 3> Rz;
     Rz << cos(y), -sin(y), 0,
         sin(y), cos(y), 0,
         0, 0, 1;

     Eigen::Matrix<Scalar_t, 3, 3> Ry;
     Ry << cos(p), 0., sin(p),
         0., 1., 0.,
         -sin(p), 0., cos(p);

     Eigen::Matrix<Scalar_t, 3, 3> Rx;
     Rx << 1., 0., 0.,
         0., cos(r), -sin(r),
         0., sin(r), cos(r);

     return Rz * Ry * Rx;
}



Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
{
     Eigen::Matrix3d R0;
     Eigen::Vector3d ng1 = g.normalized();
     Eigen::Vector3d ng2{0, 0, 1};
     // zjloc::lioOptions_m options_;//判断是否为positive模型，得到初始化后的元素
     // loadOptions(options_);
     // if (options_.imu_positive_model)
     // {
     //      ng2 = {0, 0, 1};
     //      ROS_INFO("1");
     // }
     // else
     // {
     //      ng2 = {0, 0, -1};
     //      ROS_INFO("-1");
     // }
     // Eigen::Vector3d ng2{0, 0, 1.0};//改动这个位置才能得到R
     //得到从ng1到ng2的旋转矩阵
     R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
     //旋转矩阵到欧拉角->yaw
     double yaw = R2ypr(R0).x();
     //欧拉角到旋转矩阵->R0
     R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
     // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
     return R0;
}