#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h> //common data
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

// uwb
#include "uwb_process/uwb_init_loc.h"
#include "uwb_process/utility.h"
#include "uwb_process/uwb_eskf.h"


/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {
  return (x.curvature < y.curvature);
};

const bool Compare(std::pair<size_t, PointType> &x,
                   std::pair<size_t, PointType> &y) {
  return (x.second.curvature < y.second.curvature);
};

template <class PointType, class Compare>
void argsort(PointType iterBegin, PointType iterEnd, Compare comp,
             std::vector<size_t> &indexes) {
  std::vector<std::pair<size_t, PointType>> pv;
  pv.reserve(iterEnd - iterBegin);

  PointType iter;
  size_t k;
  for (iter = iterBegin, k = 0; iter != iterEnd; iter++, k++) {
    pv.push_back(std::pair<int, PointType>(k, iter));
  }
  //    std::sort(pv.begin(), pv.end(), comp);

  std::sort(pv.begin(), pv.end(),
            [&comp](const std::pair<size_t, PointType> &a,
                    const std::pair<size_t, PointType> &b) -> bool {
              return comp(*a.second, *b.second);
            });

  indexes.resize(pv.size());
  std::transform(
      pv.begin(), pv.end(), indexes.begin(),
      [](const std::pair<size_t, PointType> &a) -> size_t { return a.first; });
}

template <typename PointType>
bool sort_indexes(
    const std::vector<PointType, Eigen::aligned_allocator<PointType>> &v) {
  // 初始化索引向量
  //    idx.reserve(v.size());
  std::vector<size_t> idx(v.size());
  //使用iota对向量赋0~？的连续值
  std::iota(idx.begin(), idx.end(), 0);
  // 通过比较v的值对索引idx进行排序
  std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {
    return v[i1].curvature < v[i2].curvature;
  });
  return true;
}

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  bool initUWB(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& ndb_msg, state_ikfom &init_state, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile);
  void set_lidar_type(const int _lidar_type);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile);
  // void Process(MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_, PointCloudXYZI::Ptr cur_pcl_un_full_, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile);

  void set_wheel_extrinsic(const V3D &transl, const M3D &rot);
  void set_wheel_nhc_cov(const V3D &scaler);
  void set_wheel_scale(const V1D &s);
  void set_uwb_extrinsic(const V3D &scaler);
  void set_uwb_nhc_cov(const V3D &scaler);
  void set_uwb_max_distance(const double& dist);
  void set_uwb_N_distance(const int& N);


  V3D cov_wheel_nhc;

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  vector<Pose6D> IMUpose;
  V3D init_position_IMU2Global;
 private:
  void IMU_init(MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile);

  // void UndistortPcl(MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out, PointCloudXYZI &pcl_out_full, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
  bool   gnss_heading_need_init_ = true;

  M3D Wheel_R_wrt_IMU;
  V3D Wheel_T_wrt_IMU;
  V1D wheel_s;

  M3D GNSS_Heading;

  // uwb
  V3D UWB_T_wrt_IMU;
  V3D cov_uwb;
  double max_dist;
  int N_dist;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());

  cov_wheel_nhc   = V3D(0.01, 0.01, 0.01);
  Wheel_T_wrt_IMU = Zero3d;
  Wheel_R_wrt_IMU = Eye3d;
  wheel_s = V1D(1.0);

  GNSS_Heading = Eye3d;

  // uwb
  UWB_T_wrt_IMU = Zero3d;
  cov_uwb = V3D(0.0001, 0.0001, 0.0001);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

// void ImuProcess::set_lidar_type(const int _lidar_type) {
//   this->lidar_type = _lidar_type;
// }

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::set_wheel_extrinsic(const V3D &transl, const M3D &rot)
{
    Wheel_T_wrt_IMU = transl;
    Wheel_R_wrt_IMU = rot;
}

void ImuProcess::set_wheel_scale(const V1D &s)
{
    wheel_s = s;
}

void ImuProcess::set_wheel_nhc_cov(const V3D &scaler)
{
    cov_wheel_nhc = scaler;
}

void ImuProcess::set_uwb_extrinsic(const V3D &scaler)
{
    UWB_T_wrt_IMU = scaler;
}

void ImuProcess::set_uwb_nhc_cov(const V3D &scaler)
{
    cov_uwb = scaler;
}

void ImuProcess::set_uwb_max_distance(const double& dist)
{
    max_dist = dist;
}

void ImuProcess::set_uwb_N_distance(const int& N)
{
    N_dist = N;
}

void ImuProcess::IMU_init(MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);

  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;

  /*** wheel velocity initialization ***/
  // wheel 初始化
  double wheel_velocity = 0.0;
  if (USE_WHEEL && !meas.wheel.empty())
  {
    ROS_INFO("init wheel!");
    wheel_velocity = meas.wheel.back()->twist.twist.linear.x;// 线速度
    V3D wheel_v_vec(wheel_velocity, 0.0, 0.0);
    M3D angv_crossmat;
    V3D real_gyr = cur_gyr - mean_gyr;
    angv_crossmat << SKEW_SYM_MATRX(real_gyr);
    init_state.vel = Wheel_R_wrt_IMU * (wheel_v_vec * wheel_s) - angv_crossmat * Wheel_T_wrt_IMU; // initial rot is identity matrix
    init_state.wheel_s = wheel_s;
    init_state.offset_R_W_I = Wheel_R_wrt_IMU;
    init_state.offset_T_W_I = Wheel_T_wrt_IMU;
  }

  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001;
  init_P(23,23) = init_P(24,24) = init_P(25,25) = 0.00001; // Wheel_R_wrt_IMU
  init_P(26,26) = init_P(27,27) = init_P(28,28) = 0.00001; // Wheel_T_wrt_IMU
  init_P(29,29) = 0.0001; // wheel scale
  init_P(30,30) = init_P(31,31) = init_P(32,32) = 0.0001; // GNSS_Heading
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
}


// 初始化UWB
bool ImuProcess::initUWB(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& ndb_msg, state_ikfom &init_state, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile)
{
    static bool ekf_inited = false;
    // ROS_INFO("pre ekf_inited = %s", ekf_inited ? "true" : "false");
    if(!ekf_inited)
    {
        // initUWB在第1帧的时候就初始化了传感器的位置，在以后的帧中都不继续初始化！
        int k_uwb_size = ndb_msg->numNeighborEntries;
        if(k_uwb_size>=3)
        {
            ekf_inited= true;
            geometry_msgs::PointStamped mobile_position_first = p_mobile->getPosition();//得到三角化后的uwb在G系下的位置p_GU
            Eigen::Vector3d first_ekf_init_position = Eigen::Vector3d(mobile_position_first.point.x, mobile_position_first.point.y, mobile_position_first.point.z);
            init_state.offset_T_U_I = UWB_T_wrt_IMU;//初始化外参
            //p_GU = p_GI + R_GI * p_IU, 将位置转换回到IMU坐标系
            init_state.pos = first_ekf_init_position - init_state.rot * init_state.offset_T_U_I;
            // init_state.pos = first_ekf_init_position - init_state.offset_T_U_I;
            ROS_WARN("INITIAL UWB pos:%.6f, %.6f, %.6f", first_ekf_init_position[0], first_ekf_init_position[1], first_ekf_init_position[2]);
            ROS_WARN("INITIAL IMU pos:%.6f, %.6f, %.6f", init_state.pos[0], init_state.pos[1], init_state.pos[2]);
            init_position_IMU2Global = init_state.pos;//设置初始化位置
            sleep(3);
            return ekf_inited;//这一段注释掉后，算法无法执行
        }
        else
        {
            std::cout<<"pls finish init in the room where enough uwb can be detected"<<std::endl;
            return false;
        }
    }
    // 初始化完成后就每次执行这个函数都返回true
    // ROS_INFO("last ekf_inited = %s", ekf_inited ? "true" : "false");
    return ekf_inited;
}


void ImuProcess::UndistortPcl(MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;

  /*** downsample wheel meas, only remain meas in the middle ***/
  if (meas.wheel.size() > 1){
      int pop_num = meas.wheel.size() / 2;
      for (int i = 0; i < pop_num; i++){
          meas.wheel.pop_front();
      }
      auto wheel_meas = meas.wheel.front();
      meas.wheel.clear();
      meas.wheel.push_back(wheel_meas);
  }

  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;

    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    // 利用acc, omega更新in
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    //向前传播
    kf_state.predict(dt, Q, in);// normal predict

    // 利用gnss更新状态
    /*** update by gnss meas ***/
    if (USE_GNSS && !meas.gnss.empty())
    {
        double gnss_time = meas.gnss.front()->header.stamp.toSec();
        if (gnss_time < head->header.stamp.toSec()){
            meas.gnss.pop_front();
        }else{
            if (gnss_time < tail->header.stamp.toSec()){ // gnss位于两个imu之间
                opt_with_gnss = true;//
                if (!gnss_heading_need_init_)
                {
                    if (meas.gnss.front()->pose.covariance[0] < 200) // 航向初始化未完成或gnss水平噪声大于200不进行更新（遮挡区域）
                    {
                        kf_state.update_iterated_dyn_share(); // gnss更新
                        ROS_WARN("gnss update !");
                    }else{
                        ROS_WARN("gnss too noise !");
                    }
                }else{
                    ROS_WARN("gnss not initialized !");
                }
                opt_with_gnss = false;
                meas.gnss.pop_front();
            }
        }
    }

    // 利用wheel更新状态
    /*** update by wheel meas ***/
    if (USE_WHEEL && !meas.wheel.empty())
    {
        double wheel_time = meas.wheel.front()->header.stamp.toSec();
        if (wheel_time < head->header.stamp.toSec()){
            meas.wheel.pop_front();
        }else{
            if (wheel_time < tail->header.stamp.toSec()){ // wheel 位于两个imu之间
                opt_with_wheel = true;
                kf_state.update_iterated_dyn_share(); // wheel更新
                ROS_WARN("wheel update !");
                opt_with_wheel = false;
                meas.wheel.pop_front();
            }
        }
    }


    // 利用uwbs更新状态
    /*** update by uwb meas ***/
    if (USE_UWB && !meas.uwb.empty())
    {
        double uwb_time = meas.uwb.front()->header.stamp.toSec();

        if (uwb_time < head->header.stamp.toSec()){
            meas.uwb.pop_front();
        }
        else
        {
          if (uwb_time < tail->header.stamp.toSec()){ // uwb 位于两个imu之间
            // ROS_INFO("start update uwb");
            opt_with_uwb = true;//uwb更新

            // 判断超过 4 * max_dist，如果是则opt_with_uwb为false, 不用uwb更新，防止运行过远后影响LIO发散
            state_ikfom init_state = kf_state.get_x();
            V3D p_IMU2Global_before_update = init_state.pos;
            double dist_current2init = (p_IMU2Global_before_update - init_position_IMU2Global).norm();

            if (dist_current2init > N_dist * max_dist)
            {
              opt_with_uwb = false;
              ROS_WARN("the distance is so large, stoping uwb update");
            }
            else
            {
              // init uwb
              // ROS_INFO("start init uwb");
              common_msgs::UWB_FullNeighborDatabase::ConstPtr ndb_msg = meas.uwb.front();//利用front(最新时间)UWB来更新系统状态
              UWB_Loc_Init initializer(*ndb_msg, p_mobile);
              if(!initializer.initializeMobileByCeres())//每帧雷达,只用最新的三角化的位置来初始化state.pos,对比之前的UWB-ESKF工作，用了每一帧的UWB数据来初始化state.pos
              {
                  ROS_ERROR("UWB is not initialized!");
                  return;
              }

              initUWB(ndb_msg, init_state, p_mobile);
              kf_state.change_x(init_state);//更新init_state,否则更新无效
              // eskf update
              kf_state.update_iterated_dyn_share(); // uwb更新
              state_ikfom updated_state = kf_state.get_x();
              V3D p_IMU2Global_after_update = updated_state.pos;
              // // p_GU = p_GI + R_GI * p_IU,求UWB在Global下的位置
              V3D p_UWB2Global = p_IMU2Global_after_update + updated_state.rot * updated_state.offset_T_U_I;

              // 重新设置uwb位置, 用于计算协方差
              p_mobile->setPosition(p_UWB2Global[0], p_UWB2Global[1], p_UWB2Global[2]);//更新position
              ROS_WARN("uwb update !");
            }

            opt_with_uwb = false;
            meas.uwb.pop_front();

          }
        }
    }

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    // 将gnss, wheel, uwb更新后的状态放入IMUpose中
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  // 再预测一次计算帧尾位姿
  // ROS_WARN("all sensors get updated!");
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  // ROS_WARN("start backward propagation");
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;

    }
  }
}


void ImuProcess::Process(MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_, boost::shared_ptr<UWB_Mobile_ESKF>& p_mobile)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    // 初始化wheel
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  // 初始化GNSS
  if (USE_GNSS && gnss_heading_need_init_){
      if (!meas.gnss.empty())
      {
          Eigen::Vector3d gnss_pose(meas.gnss.back()->pose.pose.position.x, meas.gnss.back()->pose.pose.position.y, 0.0);
          state_ikfom init_state = kf_state.get_x(); // 初始状态量
          cout << "gnss norm " << gnss_pose.norm() << endl;
          if (gnss_pose.norm() > 5)
          { // GNSS 水平位移大于5m
              Eigen::Vector3d tmp_vec(init_state.pos.x(), init_state.pos.y(), 0.0);
              GNSS_Heading = Eigen::Quaterniond::FromTwoVectors(gnss_pose, tmp_vec).toRotationMatrix();//计算gnss的朝向,让tmp_vec和gnss_pose对齐
              SO3 so3(GNSS_Heading);
              V3D euler = SO3ToEuler(so3);
              ROS_WARN_STREAM("INITIAL GNSS HEADING " << euler.transpose());
              init_state.offset_R_G_I = GNSS_Heading;//利用gnss朝向赋值给offset_R_G_I
              kf_state.change_x(init_state);//change_x
              gnss_heading_need_init_ = false;
          }
      }
  }



  // 对该帧雷达进行前向传播和反向传播
  UndistortPcl(meas, kf_state, *cur_pcl_un_, p_mobile);
  // UndistortPcl(meas, kf_state, *cur_pcl_un_, *cur_pcl_un_full_, p_mobile);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();

  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
