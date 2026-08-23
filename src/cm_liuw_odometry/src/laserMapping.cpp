// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

// CPU
#ifdef HAS_CPUID
#include <cpuid.h>
#endif
#include <ctime>
#include <iomanip>
#include <ios>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <sys/vtimes.h>
#include <thread>

// boost
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// gnss
#include "GNSS_Processing.hpp"
#include "sensor_msgs/NavSatFix.h"

// uwb
#include "uwb_process/uwb_eskf.h"
#include "uwb_process/utility.h"

#define INIT_TIME           (0.1)
// #define LASER_POINT_COV     (0.001) //暂时不使用它
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true, extrinsic_est_wheel = true, scale_est_wheel = true;
bool ros_driver_fix_en = false; //修复imu和lidar时间戳
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string saveDirectory, odom_link;
string map_file_path, lid_topic, imu_topic,  wheel_topic, ground_truth_topic;
double wheel_velocity = 0.0;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_wheel = -1.0, last_timestamp_uwb = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001, laser_point_cov = 0.001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

/*cup && lidar && ram usage message*/
int num_threads_;
std::string cpu_type;
std::vector<double> cpu_percents;
clock_t lastCPU, lastSysCPU, lastUserCPU;
int numProcessors;

//time_stamp
ros::Time scan_header_stamp;
double scan_stamp;
double prev_scan_stamp;
double scan_dt;
std::vector<double> comp_times;
std::vector<double> imu_rates;
std::vector<double> lidar_rates;

// Trajectory
double length_traversed;
Eigen::Vector3f origin;//开始位置
/*cup && lidar && ram usage message*/

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
vector<double>       extrinT_wheel(3, 0.0);
vector<double>       extrinR_wheel(9, 0.0);
vector<double>       wheel_scale(1, 1.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<nav_msgs::OdometryConstPtr> wheel_buffer;
deque<common_msgs::UWB_FullNeighborDatabase::ConstPtr> uwb_buffer;//uwb_buffer

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort_full(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
V3D Wheel_T_wrt_IMU(Zero3d);
M3D Wheel_R_wrt_IMU(Eye3d);
V1D wheel_s(1.0);


/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Path path_ground_truth;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// gnss
double last_timestamp_gnss = -1.0 ;
deque<nav_msgs::Odometry::Ptr> gnss_buffer;
geometry_msgs::PoseStamped msg_gnss_pose;
geometry_msgs::Vector3 msg_gnss_cov;
string gnss_topic ;
ros::Publisher pubGnssCov;

M3D Gnss_R_wrt_IMU(Eye3d) ;         // gnss  与 imu 的外参
V3D Gnss_T_wrt_IMU(Zero3d);
M3D GNSS_Heading(Eye3d);
bool gnss_inited = false ;                        //  是否完成gnss初始化
shared_ptr<GnssProcess> p_gnss(new GnssProcess());
GnssProcess gnss_data;
ros::Publisher pubGnssPath ;
nav_msgs::Path gps_path ;
vector<double>       extrinT_Gnss2IMU(3, 0.0);
vector<double>       extrinR_Gnss2IMU(9, 0.0);


/*uwb*/
geometry_msgs::PoseStamped msg_uwb_pose;
geometry_msgs::Vector3 msg_uwb_cov;

string uwb_topic;
double m_slam_fps;
bool m_is_fix_fps;
bool m_is_initialize_with_ceres;
bool is_runtime_output;
bool is_pub_position_covariance;
bool is_pub_observation_vector;
double position_cov_marker_scale;
double m_td;
double m_current_time;
double first_uwb_time;

// ## with uwb ekf #####
double z_damping_factor;
double Q_scale;
double R_scale;
double tao_acc_sqrt;
double tao_bias_sqrt;
double initialization_innovation_threshold;
double normal_innovation_threshold;
double innovation_threshold_inc_on_succ;
double innovation_threshold_inc_on_fail;
double kalman_sigma_a;
// ## with uwb ekf #####


M3D UWB_R_wrt_IMU(Eye3d) ;         //uwb  与 imu 的外参
V3D UWB_T_wrt_IMU(Zero3d);
bool uwb_inited = false;                        //  是否完成gnss初始化
vector<double>       extrinT_UWB2IMU(3, 0.0);

V3D first_ekf_init_position;
V3D second_ekf_init_position;
bool init_flag=false;

ros::Publisher m_PositionWithCovariance_pub;//position_cov
ros::Publisher m_covariances_publisher;//all_cov

visualization_msgs::Marker uwb_station_marker;//发布uwb_station_marker
ros::Publisher uwb_station_marker_pub;
ros::Publisher pubUWBPositionMarker;//cov position
ros::Publisher pubPositionWithCovariance;

sensor_msgs::ImuConstPtr last_imu_;//上一帧imu(record the cur imu data)
int m_mobile_id;
boost::shared_ptr<UWB_Mobile_ESKF> p_mobile;
geometry_msgs::Pose m_position;//当前geometry方法的position
geometry_msgs::Twist m_velocity;
Eigen::Matrix3d m_position_covariance;//position的cov
std::vector<int> m_anchor_list;
std::map<int, boost::shared_ptr<UWB_Anchor_ESKF> > m_anchor_map;//anchor地图映射
int index_frame;//帧
std::vector<V3D> vec_anchor;
double max_distance;//UWB最大间隔
/*uwb*/

/*observation matrix*/
Eigen::Matrix<double, 6, 6> A_p = Eigen::Matrix<double, 6, 6>::Zero();
ros::Publisher pub_eigen_marker;
ros::Publisher pub_observability_vector;
/*observation matrix*/


PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

string pcd_save_dir, pcd_name, scene_name;

void SigHandle(int sig)
{
    // if (pcl_wait_save->size() > 0 && pcd_save_en) {
    //     // 下采样
    //     //    pcl::PointCloud<PointType>::Ptr pc_filter(new
    //     //    pcl::PointCloud<PointType>()); pcl::VoxelGrid<PointType> sor;
    //     //    sor.setLeafSize(map_filter_size, map_filter_size, map_filter_size);
    //     //    sor.setInputCloud(pcl_wait_save);
    //     //    sor.filter(*pc_filter);
    //     //    pc_filter->height = 1;
    //     //    pc_filter->resize(pc_filter->points.size());

    //     string pcd_dir(pcd_save_dir + string(pcd_name + ".pcd"));
    //     try {
    //         pcl::PCDWriter pcd_writer;
    //         pcd_writer.writeBinary(pcd_dir, *pcl_wait_save);
    //         cout << "current scan saved to : " << pcd_dir << ", "
    //              << pcl_wait_save->points.size() << endl;
    //     } catch (std::exception e) {
    //         ROS_ERROR("SAVE PCD ERROR", pcl_wait_save->points.size());
    //     }
    // }

    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count ++;

    scan_stamp = msg->header.stamp.toSec();//这一帧雷达时间
    prev_scan_stamp = last_timestamp_lidar;//上一帧雷达时间
    lidar_rates.push_back( 1. / (scan_stamp - prev_scan_stamp) );//雷达帧率

    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();

    scan_stamp = msg->header.stamp.toSec();//这一帧雷达时间
    prev_scan_stamp = last_timestamp_lidar;//上一帧雷达时间
    lidar_rates.push_back( 1. / (scan_stamp - prev_scan_stamp) );//雷达帧率

    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    prev_scan_stamp = scan_stamp;

    last_timestamp_lidar = msg->header.stamp.toSec();//更新这一帧雷达时间

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// 原始的imu_cbk
// void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
// {
//     publish_count ++;
//     // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
//     sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

//     msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
//     if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
//     {
//         msg->header.stamp = \
//         ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
//     }

//     double timestamp = msg->header.stamp.toSec();

//     mtx_buffer.lock();

//     if (timestamp < last_timestamp_imu)
//     {
//         ROS_WARN("imu loop back, clear buffer");
//         imu_buffer.clear();
//     }

//     last_timestamp_imu = timestamp;
//     last_imu_ = msg;
//     imu_buffer.push_back(msg);
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }

// 更新后imu_cbk
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count ++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // 1. 减去固定的时间偏移（如果有）
    double timestamp = msg_in->header.stamp.toSec() - time_diff_lidar_to_imu;

    // 2. 可选：基于 timediff_lidar_wrt_imu 的动态同步（保留你原来的代码，但注意不要重复）
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        timestamp = timediff_lidar_wrt_imu + msg_in->header.stamp.toSec();
    }

    // 3. 记录 IMU 速率（注意 dt 计算需要基于上一帧的 timestamp，不要用修正前的）
    double dt = timestamp - last_timestamp_imu;
    if (dt <= 0) dt = 0.005;  // 默认 200Hz
    imu_rates.push_back(1.0 / dt);

    // 4. 修正时间戳以使 IMU 和 LiDAR 时间对齐（关键安全修改）
    if (ros_driver_fix_en && last_timestamp_lidar > 0.0) {
        double diff = last_timestamp_lidar - timestamp;
        // 只修正差值在合理范围内（例如 ±10 秒），避免因初始值或异常数据导致巨大偏移
        if (fabs(diff) < 50.0) {
            timestamp += std::round(diff);   // 对齐到最近的整数倍（假设 IMU 频率是 LiDAR 的整数倍）
        } else {
            ROS_WARN("Large timestamp diff (%.2f s) between IMU and LiDAR, skip correction", diff);
        }
    }

    // 5. 检查修正后的时间戳是否有效（正数、有限）
    if (timestamp <= 0 || !std::isfinite(timestamp)) {
        ROS_ERROR("Invalid IMU timestamp: %.10f, discarding", timestamp);
        return;
    }

    // 6. 设置消息的 ROS 时间（这里已经确保 timestamp 是有效值）
    msg->header.stamp = ros::Time().fromSec(timestamp);

    // 7. 处理 IMU 与 LiDAR 不同步警告（不修改时间，仅警告）
    if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en)) {
        ROS_WARN("IMU and LiDAR not synced! delta time: %.3lf", last_timestamp_lidar - timestamp);
    }

    // 8. 线程安全地存入 buffer
    mtx_buffer.lock();
    if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu) {
        mtx_buffer.unlock();
        ROS_ERROR("imu loop back, offset: %.6lf, discarding", last_timestamp_imu - timestamp);
        return;
    }
    last_timestamp_imu = timestamp;
    last_imu_ = msg;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void wheel_cbk(const nav_msgs::OdometryConstPtr &msg)
{
    // cout<<"Wheel got at: "<<msg_in->header.stamp.toSec()<<endl;

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_wheel)
    {
        ROS_WARN("wheel loop back, clear buffer");
        wheel_buffer.clear();
    }

    last_timestamp_wheel = timestamp;

    wheel_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void gnss_cbk(const sensor_msgs::NavSatFixConstPtr& msg_in)
{
    //  ROS_INFO("GNSS DATA IN ");
    double timestamp = msg_in->header.stamp.toSec();

    mtx_buffer.lock();

    // 没有进行时间纠正
    if (timestamp < last_timestamp_gnss)
    {
        ROS_WARN("gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = timestamp;
    gnss_data.status = msg_in->status.status;
    gnss_data.service = msg_in->status.service;
    gnss_data.pose_cov[0] = msg_in->position_covariance[0];
    gnss_data.pose_cov[1] = msg_in->position_covariance[4];
    gnss_data.pose_cov[2] = msg_in->position_covariance[8];

    mtx_buffer.unlock();

    // UWB也得进行初始化
    if(!gnss_inited){           //  初始化位置
        gnss_data.InitOriginPosition(msg_in->latitude, msg_in->longitude, msg_in->altitude) ;
        gnss_inited = true ;
    }else{                               //   初始化完成
        gnss_data.UpdateXYZ(msg_in->latitude, msg_in->longitude, msg_in->altitude) ;             //  WGS84 -> ENU

        Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
        gnss_pose(0,3) = gnss_data.local_E ;                 //    东
        gnss_pose(1,3) = gnss_data.local_N ;                 //     北
        gnss_pose(2,3) = gnss_data.local_U ;                 //    天

        Eigen::Isometry3d trans(Gnss_R_wrt_IMU) ;
        trans.pretranslate(Gnss_T_wrt_IMU);//
        // p_I = T_I_GN *  p_GN
        gnss_pose  =  trans.inverse()  *  gnss_pose;  // convert gnss_pose to IMU frame

        nav_msgs::Odometry::Ptr gnss_data_enu(new nav_msgs::Odometry());
        // add new message to buffer:将p_I转换为odom,并加入到gnss_buffer中
        gnss_data_enu->header.stamp = ros::Time().fromSec(gnss_data.time);
        gnss_data_enu->pose.pose.position.x =  gnss_pose(0,3) ;  //gnss_data.local_E ;   东
        gnss_data_enu->pose.pose.position.y =  gnss_pose(1,3) ;  //gnss_data.local_N;    北
        gnss_data_enu->pose.pose.position.z =  gnss_pose(2,3) ;  //  天

        gnss_data_enu->pose.pose.orientation.x =  geoQuat.x ;                //  gnss 的姿态不可观，所以姿态只用于可视化，取自imu
        gnss_data_enu->pose.pose.orientation.y =  geoQuat.y;
        gnss_data_enu->pose.pose.orientation.z =  geoQuat.z;
        gnss_data_enu->pose.pose.orientation.w =  geoQuat.w;

        gnss_data_enu->pose.covariance[0] = gnss_data.pose_cov[0] ;
        gnss_data_enu->pose.covariance[7] = gnss_data.pose_cov[1] ;
        gnss_data_enu->pose.covariance[14] = gnss_data.pose_cov[2] ;

        gnss_buffer.push_back(gnss_data_enu);

        // visual gnss path in rviz:
        msg_gnss_pose.header.frame_id = "camera_init";
        msg_gnss_pose.header.stamp = ros::Time().fromSec(gnss_data.time);

        Eigen::Vector3d gnss_vec(gnss_pose(0,3), gnss_pose(1,3), gnss_pose(2,3));
        gnss_vec = GNSS_Heading * gnss_vec;//p_G = R_GI * p_I, 利用R_GI将p_I转换到G系下

        msg_gnss_pose.pose.position.x = gnss_vec.x() ;
        msg_gnss_pose.pose.position.y = gnss_vec.y() ;
        msg_gnss_pose.pose.position.z = gnss_vec.z() ;

        gps_path.poses.push_back(msg_gnss_pose);//发布gnss姿态

        msg_gnss_cov.x = gnss_data.pose_cov[0];
        msg_gnss_cov.y = gnss_data.pose_cov[1];
        msg_gnss_cov.z = gnss_data.pose_cov[2];

        pubGnssCov.publish(msg_gnss_cov);
    }
}


// 有效的uwb database
bool getValidNeighborDatabse(const common_msgs::UWB_FullNeighborDatabase &ndb)
{
    int valid_anchor_counter = 0;

    for(int i=0;i<ndb.numNeighborEntries;++i)
    {
        geometry_msgs::PointStamped anchor_position;
        // 输出anchorId对应的range
        // std::cout<<ndb.neighbors.at(i).nodeId<<", "<<ndb.neighbors.at(i).rangeMm<<std::endl;
        if(p_mobile->getAnchorPosition(ndb.neighbors.at(i).nodeId, anchor_position)
                && ndb.neighbors.at(i).rangeMm/1000.0>0.3) //用米制来表示
        {
            valid_anchor_counter += 1;
        }
        // ROS_INFO("[Ceres Init] Total ndb anchor: %d", valid_anchor_counter);
    }

    if(valid_anchor_counter < p_mobile->m_least_anchor_num)
    {
        return false;
    }else
    {
        return true;
    }
}

void uwb_cbk(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& msg)
{
    common_msgs::UWB_FullNeighborDatabase m_ndb = *msg;

    if(getValidNeighborDatabse(m_ndb))
    {
        double timestamp = msg->header.stamp.toSec();

        mtx_buffer.lock();
        if (timestamp < last_timestamp_uwb)
        {
            ROS_WARN("uwb loop back, clear buffer");
            uwb_buffer.clear();
        }

        last_timestamp_uwb = timestamp;

        uwb_buffer.push_back(msg);

        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;

//同步时间
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    if (USE_WHEEL && wheel_buffer.empty()){
        opt_with_wheel = false;
    }

    if (USE_GNSS && gnss_buffer.empty()){
        opt_with_gnss = false;
    }

    // 使用uwb
    if (USE_UWB && uwb_buffer.empty()){
        opt_with_uwb = false;//uwb_buffer为空的时候直接置为false
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        //lidarPoint
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();//开始时间
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;//te = tb + delta_t(雷达平均扫描时间)
            ROS_WARN("Too few input point cloud!\n");
        }//若帧尾点时间 < 0.5 * T
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);//开始时间戳+结尾点时间戳
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;//雷达平均扫描时间
        }

        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }

    //最后imu时间戳一定 >  lidar_end_time
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    // /*** find the closet wheel frame to the last imu frame ***/
    if (USE_WHEEL && !wheel_buffer.empty())
    {
        meas.wheel.clear();
        double wheel_time = wheel_buffer.front()->header.stamp.toSec();
        while ((!wheel_buffer.empty()) && (wheel_time < lidar_end_time)) //记录wheel数据，wheel时间小于当前帧lidar结束时间
        {
            wheel_time = wheel_buffer.front()->header.stamp.toSec();
            if (wheel_time > lidar_end_time)
                break;
            meas.wheel.push_back(wheel_buffer.front()); //记录当前lidar帧内的wheel数据到meas.wheel
            wheel_buffer.pop_front();
        }
    }

    /*** find the closet gnss frame to the last imu frame ***/
    if (USE_GNSS && !gnss_buffer.empty())
    {
        meas.gnss.clear();
        double gnss_time = gnss_buffer.front()->header.stamp.toSec();
        while ((!gnss_buffer.empty()) && (gnss_time < lidar_end_time)) //记录gnss数据，gnss时间小于当前帧lidar结束时间
        {
            gnss_time = gnss_buffer.front()->header.stamp.toSec();
            if (gnss_time > lidar_end_time)
                break;
            meas.gnss.push_back(gnss_buffer.front()); //记录当前lidar帧内的gnss数据到meas.gnss
            gnss_buffer.pop_front();
        }
    }

    /*** find the closet uwb frame to the last imu frame ***/
    // 找到离last imu frame 最近的uwb frame，并加入到meas.uwb中
    if (USE_UWB && !uwb_buffer.empty())
    {
        meas.uwb.clear();
        double uwb_time = uwb_buffer.front()->header.stamp.toSec();
        while ((!uwb_buffer.empty()) && (uwb_time < lidar_end_time)) //记录uwb数据，uwb时间小于当前帧lidar结束时间
        {
            uwb_time = uwb_buffer.front()->header.stamp.toSec();
            if (uwb_time > lidar_end_time)
                break;
            meas.uwb.push_back(uwb_buffer.front()); //将 < lidar_end_time的所有uwb加入到meas.uwb中
            uwb_buffer.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

// PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
// PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        //发布点云
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        // 如果pcd_save_interval  = 0，则会保存每一帧的点云，否则只会保存最后一帧点云
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            // string all_points_dir(pcd_save_dir + string(pcd_name + ".pcd"));
            pcl::PCDWriter pcd_writer;
            ROS_WARN("current scan saved to /PCD/: %s", all_points_dir.c_str());
            // cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }


}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body_ori";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

// void publish_frame_body_raw(const ros::Publisher &pubLaserCloudFull_body_raw) {
//     // full body points
//     sensor_msgs::PointCloud2 laserCloudmsgRaw;
//     // pcl::PointCloud<pcl::PointXYZI> temp_cloud;
//     int size = feats_undistort_full->points.size();

//     PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

//     for (int i = 0; i < size; i++) {
//         //        pcl::PointXYZI p_body_lidar;
//         //        p_body_lidar.x = feats_undistort_full->points[i].x;
//         //        p_body_lidar.y = feats_undistort_full->points[i].y;
//         //        p_body_lidar.z = feats_undistort_full->points[i].z;
//         //        p_body_lidar.intensity =
//         //        feats_undistort_full->points[i].intensity;
//         //        temp_cloud.push_back(p_body_lidar);
//         RGBpointBodyLidarToIMU(&feats_undistort_full->points[i],
//                                &laserCloudIMUBody->points[i]);
//     }
//     // temp_cloud.resize(size);

//     pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsgRaw);
//     laserCloudmsgRaw.header.stamp = ros::Time().fromSec(lidar_end_time);
//     laserCloudmsgRaw.header.frame_id = "body_ori";
//     pubLaserCloudFull_body_raw.publish(laserCloudmsgRaw);
//     publish_count -= PUBFRAME_PERIOD;

//     //    pcl::toROSMsg(temp_cloud, laserCloudmsgRaw);
//     //    laserCloudmsgRaw.header.stamp = ros::Time().fromSec(lidar_end_time);
//     //    laserCloudmsgRaw.header.frame_id = "body";
//     //    pubLaserCloudFull_body_raw.publish(laserCloudmsgRaw);
//     //    publish_count -= PUBFRAME_PERIOD;
// }

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body_ori";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    // ###### lidar link, added from paloc ######
    // state value, pose, velocity, gravity, bias
    state_ikfom curr_state = kf.get_x();
    odomAftMapped.twist.twist.linear.x = curr_state.vel[0];//线速度
    odomAftMapped.twist.twist.linear.y = curr_state.vel[1];
    odomAftMapped.twist.twist.linear.z = curr_state.vel[2];

    odomAftMapped.twist.covariance[1] = curr_state.ba[0];//角速度
    odomAftMapped.twist.covariance[2] = curr_state.ba[1];
    odomAftMapped.twist.covariance[3] = curr_state.ba[2];
    odomAftMapped.twist.covariance[4] = curr_state.bg[0];
    odomAftMapped.twist.covariance[5] = curr_state.bg[1];
    odomAftMapped.twist.covariance[6] = curr_state.bg[2];
    odomAftMapped.twist.covariance[8] = curr_state.grav[0];
    odomAftMapped.twist.covariance[9] = curr_state.grav[1];
    odomAftMapped.twist.covariance[10] = curr_state.grav[2];
    // ###### lidar link, added from paloc ######

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body_ori" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

//  发布gnss 轨迹
void publish_gnss_path(const ros::Publisher pubPath)
{
    gps_path.header.stamp = ros::Time().fromSec(lidar_end_time);
    gps_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        pubPath.publish(gps_path);
    }
}

// void ground_truth_cbk(const nav_msgs::Odometry::ConstPtr& odom_msg)
// {
//     geometry_msgs::PoseStamped pose_stamped;
//     // 设置pose的时间戳和坐标系
//     pose_stamped.header.stamp = odom_msg->header.stamp;
//     pose_stamped.header.frame_id = "camera_init";

//     // 从里程计中提取位姿
//     pose_stamped.pose = odom_msg->pose.pose;

//     // 将位姿添加到路径中
//     path_ground_truth.poses.push_back(pose_stamped);

//     // 设置路径的时间戳
//     path_ground_truth.header.stamp = odom_msg->header.stamp;
//     path_ground_truth.header.frame_id = "camera_init";

//     // 发布路径
//     path_ground_truth.publish(path_msg_);
// }


// // ============= //
//  ######## h_share_simulation ######
// void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
// {
//     // gnss的观测方程构建
//     if (opt_with_gnss){
//         // ROS_INFO("h_share_model start opt_with_gnss");
//         ekfom_data.z = MatrixXd::Zero(3, 1);//z为名义观测
//         ekfom_data.h_x = MatrixXd::Zero(3, 36);//hx为含delta_x和noise的观测方程
//         ekfom_data.h.resize(3);
//         ekfom_data.R = MatrixXd::Zero(3, 3);//观测误差
//         ekfom_data.h_v = MatrixXd::Identity(3, 3);//观测noise
//         // residual (estimate heading)
//         V3D gnss_pos(Measures.gnss.front()->pose.pose.position.x, Measures.gnss.front()->pose.pose.position.y, Measures.gnss.front()->pose.pose.position.z);
//         V3D res = s.offset_R_G_I * gnss_pos - s.pos;//误差量：观测 - 预测(gnss_pos为gnss在G系下的位置, 利用offset_R_G_I将其转移到I系在G系下的位置)
//         ekfom_data.h(0) = res.x();
//         ekfom_data.h(1) = res.y();
//         ekfom_data.h(2) = res.z();
//         // jacobian (estimate heading)
//         ekfom_data.h_x.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity(); // d_dp
//         M3D crossmat;
//         crossmat << SKEW_SYM_MATRX(gnss_pos);
//         auto P = kf.get_P();
//         cout << "roll std " << sqrt(P(30,30)) << " pitch std" << sqrt(P(31, 31)) << " yaw std " << sqrt(P(32, 32)) << endl;
//         // if yaw std converges do not estimate R_G_I
//         if (sqrt(P(32, 32)) > 1e-4)
//         {
//             ekfom_data.h_x.block<3, 3>(0, 30) = -(s.offset_R_G_I * crossmat); // d_dR_G_I
//             ROS_WARN("Estimate R_G_I !");
//         }
//         // covariance
//         ekfom_data.R(0, 0) = 0.01 * Measures.gnss.front()->pose.covariance[0];
//         ekfom_data.R(1, 1) = 0.01 * Measures.gnss.front()->pose.covariance[7];
//         ekfom_data.R(2, 2) = Measures.gnss.front()->pose.covariance[14];
//         // 当GNSS或者WHEEL的h和hx构建完后，直接return, 不进行lidar构建
//         // 然后将opt_with_GNSS和opt_with_wheel置为false
//         return;
//     }

//     // 轮速计观测方程构建
//     // 当且仅当opt_with_wheel为true,即wheel时间戳在head, tail之间时，利用wheel来进行更新
//     // 更新完opt_with_wheel置为false，不影响其他传感器的观测
//     if (opt_with_wheel){
//         // ROS_INFO("h_share_model start opt_with_wheel");
//         ekfom_data.z = MatrixXd::Zero(3, 1);
//         ekfom_data.h_x = MatrixXd::Zero(3, 36);
//         ekfom_data.h.resize(3);
//         ekfom_data.R = MatrixXd::Zero(3, 3);
//         ekfom_data.h_v = MatrixXd::Identity(3, 3);
//         // residual
//         M3D angv_crossmat;
//         V3D gyr = kf.get_input().gyro;//gyr
//         V3D gyr_vec(gyr[0] - s.bg(0), gyr[1] - s.bg(1), gyr[2] - s.bg(2));//gyr_vec = gyr - bg
//         angv_crossmat << SKEW_SYM_MATRX(gyr_vec);
//         wheel_velocity = Measures.wheel.front()->twist.twist.linear.x;//前进速度
//         V3D wheel_v_vec(wheel_velocity,0.0,0.0);

//         // s.wheel_s 是一个尺度因子（scale factor），用于校正车轮速度的测量值。
//         // ROS_WARN_STREAM("wheel_velocity " << wheel_velocity);
//         V3D res = wheel_v_vec * s.wheel_s - s.offset_R_W_I.toRotationMatrix().transpose() * (s.rot.toRotationMatrix().transpose() * s.vel + angv_crossmat * s.offset_T_W_I);
//         // ROS_WARN_STREAM("res " << res.transpose());
//         ekfom_data.h(0) = res.x();
//         ekfom_data.h(1) = res.y();
//         ekfom_data.h(2) = res.z();
//         // jacobian
//         M3D rot_crossmat;
//         V3D tmp_vel = s.rot.toRotationMatrix().transpose() * s.vel;
//         rot_crossmat << SKEW_SYM_MATRX(tmp_vel); // 当前状态imu系下 点坐标反对称矩阵
//         ekfom_data.h_x.block<3, 3>(0,3) = -s.offset_R_W_I.toRotationMatrix().transpose() * rot_crossmat; // diff w.r.t. rot
//         ekfom_data.h_x.block<3, 3>(0,12) = -s.offset_R_W_I.toRotationMatrix().transpose() * s.rot.toRotationMatrix().transpose(); // diff w.r.t. vel
//         M3D bg_crossmat;
//         bg_crossmat << SKEW_SYM_MATRX(s.offset_T_W_I);
//         ekfom_data.h_x.block<3, 3>(0,15) = -s.offset_R_W_I.toRotationMatrix().transpose() * bg_crossmat; // diff w.r.t. bg
//         if (extrinsic_est_wheel){
//             V3D tmp_vec = s.offset_R_W_I.toRotationMatrix().transpose() * (s.rot.toRotationMatrix().transpose() * s.vel + angv_crossmat * s.offset_T_W_I);
//             M3D ex_rot_crossmat;
//             ex_rot_crossmat << SKEW_SYM_MATRX(tmp_vec);
//             ekfom_data.h_x.block<3, 3>(0,23) = -ex_rot_crossmat;
//             ekfom_data.h_x.block<3, 3>(0,26) = -s.offset_R_W_I.toRotationMatrix().transpose() * angv_crossmat;
//         }
//         if (scale_est_wheel){
//             ekfom_data.h_x.block<3, 1>(0,29) = wheel_v_vec;
//         }

//         // std::cout << "[ wheel ]: h_x: \n" << std::endl;
//         // std::cout << ekfom_data.h_x << std::endl;

//         // covariance
//         Eigen::Matrix3d tmp_mat = s.offset_R_W_I.toRotationMatrix().transpose() * bg_crossmat;
//         Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Identity();
//         cov_mat(0, 0) = wheel_cov;
//         if (gyr_vec.norm() > 0.3){
//             cov_mat(1, 1) = wheel_velocity * gyr_vec.norm();
//         }else{
//             cov_mat(1, 1) = nhc_y_cov;
//         }
//         cov_mat(2, 2) = nhc_z_cov;
//         cov_mat = cov_mat + tmp_mat * tmp_mat.transpose() * gyr_cov;
//         ekfom_data.R = cov_mat;
//         return;
//     }

//     // uwb观测方程构建

//     if (opt_with_uwb){
//         // ROS_INFO("h_share model start opt_with_uwb");
//         int k_uwb_size = p_mobile->m_ranges_to_anchors.size();
//         // int k_uwb_size=current_uwb_data_.connected_nodes.size();//uwb个数
//         // std::cout<<"received all uwb stations "<<k_uwb_size<<std::endl;
//         int all_observation_size = 0;
//         all_observation_size = k_uwb_size+3; // 3 for non_holonomic constrian

//         ekfom_data.z = MatrixXd::Zero(all_observation_size, 1);
//         ekfom_data.h_x = MatrixXd::Zero(all_observation_size, 36);
//         ekfom_data.h.resize(all_observation_size);
//         ekfom_data.R = MatrixXd::Zero(all_observation_size, all_observation_size);
//         ekfom_data.h_v = MatrixXd::Identity(all_observation_size, all_observation_size);

//         state_point = kf.get_x();//得到当前状态,暂时不用定义
//         if(0==k_uwb_size){
//             std::cout<<"no workable uwb stationes"<<std::endl;
//         }
//         else
//         {
//             // 所有观测的size
//             Eigen::MatrixXd Y(all_observation_size,1);
//             Eigen::MatrixXd Gt(all_observation_size, 36);//  Dynamic matrix
//             // init Gt (n * 15)
//             for(int m=0;m<all_observation_size;m++){
//                 for(int n=0;n<36;n++){
//                     Gt(m,n)=0;
//                 }
//             }
//             uwb_station_marker.points.clear();
//             //进行滤波更新

//             // ROS_INFO("start uwb update sharemodel");
//             std::map<int, common_msgs::UWB_FullRangeInfo>::iterator iter;
//             int i = 0;
//             for(iter=p_mobile->m_ranges_to_anchors.begin();iter!= p_mobile->m_ranges_to_anchors.end() && i < k_uwb_size;++iter, ++i)
//             {
//                 int anchor_id = iter->first;
//                 geometry_msgs::PointStamped anchor_position;
//                 /*debug code*/
//                 // bool is_get_anchor = p_mobile->getAnchorPosition(anchor_id, anchor_position);
//                 // ROS_INFO("is_get_anchor = %s", is_get_anchor ? "true" : "false");
//                 // double anchor_range = p_mobile->getRangeMeterToAnchor(anchor_id);
//                 // ROS_INFO("anchor id: %d, get_from_anchor_range: %f", anchor_id, anchor_range);
//                 // ROS_INFO("anchor_position: %f, %f, %f", anchor_position.point.x, anchor_position.point.y, anchor_position.point.z);
//                 if(p_mobile->getAnchorPosition(anchor_id, anchor_position))
//                 {
//                     // std::cout << "getAnchorPosition beg" << std::endl;
//                     // int current_uwb_station_id = ndb_msg->neighbors.at(i).nodeId;
//                     V3D tag_position;//p_GU
//                     V3D position_I2G = state_point.pos;//p_GI
//                     tag_position = state_point.pos + state_point.rot * state_point.offset_T_U_I;//p_GU = p_GI + RGI * p_IU
//                     V3D anchor_pos_eigen = V3D(anchor_position.point.x
//                                                                 ,anchor_position.point.y
//                                                                 ,anchor_position.point.z);
//                     // V3D temp_uwb_dis=tag_position-init_position_class.uwb_stations_position[current_uwb_station_id];
//                     V3D temp_uwb_dis=tag_position - anchor_pos_eigen;//先验约束

//                     //std::cout<<"uwb_stations_position "<<init_position_class.uwb_stations_position[current_uwb_station_id]<<std::endl;
//                     //std::cout<<"tag_position "<<tag_position<<std::endl;
//                     //std::cout<<"t emp_uwb_dis "<<temp_uwb_dis<<std::endl;
//                     //////////////////////
//                     geometry_msgs::Point p;
//                     p.x = anchor_position.point.x;
//                     p.y = anchor_position.point.y;
//                     p.z = anchor_position.point.z;

//                     uwb_station_marker.points.push_back(p);//发布marker
//                     ////////////////////////
//                     // temp_uwb_dis(2,0)=0;////////////////////
//                     double prior_dis=temp_uwb_dis.norm();
//                     double observation_dis = iter->second.precisionRangeMm/1000.0;

//                     //std::cout<<"current_uwb_station_id "<<current_uwb_station_id<<std::endl;
//                     //std::cout<<"uwb_observation_dis "<<observation_dis<<std::endl;
//                     double delta_dis = observation_dis - prior_dis;
//                     Y(i, 0)= delta_dis;//距离约束dist
//                     // std::cout<<"delta_dis "<<delta_dis << std::endl;
//                     // V3D error_state_delta_pos;
//                     // error_state_delta_pos<<error_state_.x(0,0),error_state_.x(1,0),error_state_.x(2,0);//errror state
//                     // V3D dis_vector= anchor_pos_eigen - error_state_delta_pos - tag_position;//range和nominalrange的差值
//                     V3D dis_vector= anchor_pos_eigen - tag_position;//range和nominalrange的差值
//                     // dis_vector(2,0)=0;/////////////////////////
//                     M3D ex_U2I_crossmat;
//                     ex_U2I_crossmat << SKEW_SYM_MATRX(state_point.offset_T_U_I);//UWB2IMU外参
//                     Gt.block<1, 3>(i, 0) = dis_vector.transpose()*(1/dis_vector.norm());
//                     // 去掉r_U对R_GI的约束,否则会扰乱系统的旋转，即旋转后点云也旋转了
//                     // Gt.block<1, 3>(i, 3) = -(1/dis_vector.norm()) * dis_vector.transpose() * state_point.rot.toRotationMatrix() * ex_U2I_crossmat;
//                     // std::cout<<"dis_vector.norm() = "<<dis_vector.norm()<<std::endl;
//                     // std::cout << "getAnchorPosition end" << std::endl;
//                 }
//             }

//             uwb_station_marker_pub.publish(uwb_station_marker);

//             /*non holonomic constrain part(非整体约束部分)
//             // V3D R_i_z(0,0,1);
//             // V3D R_w_z(0,0,1);
//             // Y.block<3, 1>(k_uwb_size, 0)= state_point.rot * R_i_z - R_w_z;//将非一致性约束加入到观测方程中
//             // V3D temp_non_holo_vector = state_point.rot * R_i_z;
//             // M3D temp_non_holo_matrix;
//             // temp_non_holo_matrix<<0,-temp_non_holo_vector(2),temp_non_holo_vector(1),temp_non_holo_vector(2),
//             //                         0,-temp_non_holo_vector(0),-temp_non_holo_vector(1),temp_non_holo_vector(0),0;
//             // 这里务必去掉homo约束，否则会导致p_GI, R_GI发生变化，从而导致body发生旋转+点云变厚（Y轴发生偏移）
//             // Gt.block<3, 3>(k_uwb_size, 3) = temp_non_holo_matrix;*/

//             Eigen::MatrixXd Ct(all_observation_size, all_observation_size);
//             //init Ct(h_v)
//             for(int m=0;m<all_observation_size;m++){
//                 for(int n=0;n<all_observation_size;n++){
//                     if(m==n){
//                         Ct(m,n)=1;
//                     }
//                     else{
//                         Ct(m,n)=0;
//                     }
//                 }
//             }

//             //init R(v~N(0, R)),观测噪声
//             Eigen::MatrixXd R(all_observation_size, all_observation_size);
//             for(int m=0;m<all_observation_size;m++){
//                 for(int n=0;n<all_observation_size;n++){
//                     if(m==n){
//                         R(m,n)=uwb_noise_* uwb_noise_;// UWB noise
//                     }
//                     else{
//                         R(m,n)=0;
//                     }
//                 }
//             }

//             // ROS_INFO("end uwb update sharemodel");
//             // non_holonomic noise
//             R(k_uwb_size,k_uwb_size)=non_hol_noise_z_*non_hol_noise_z_;
//             R(k_uwb_size+1,k_uwb_size+1)=non_hol_noise_z_*non_hol_noise_z_;
//             R(k_uwb_size+2,k_uwb_size+2)=non_hol_noise_z_*non_hol_noise_z_;//非一致性约束噪声

//             //std::cout<<"Ct "<<Ct<<std::endl;
//             //std::cout<<"R "<<R<<std::endl;

//             // update ekfom_data
//             // ROS_INFO("update ekfom_data");
//             ekfom_data.h = Y;
//             ekfom_data.h_x = Gt;
//             ekfom_data.R = R;//R != 0, otherwise,the system is going to crash.
//             ekfom_data.h_v = Ct;//h_v is identity alike
//         }
//         return;
//     }


//     double match_start = omp_get_wtime();
//     laserCloudOri->clear();
//     corr_normvect->clear();
//     total_residual = 0.0;

//     /** closest surface search and residual computation **/
//     // 临近平面搜索和残差构建
//     // 当执行update_iterated_dyn_share_modified时,然后将opt_with_GNSS和opt_with_wheel都为false，仅执行点面残差观测h,hx
//     #ifdef MP_EN
//         omp_set_num_threads(MP_PROC_NUM);
//         #pragma omp parallel for
//     #endif
//     // 构建feats_down_size个点的点面残差
//     // ROS_INFO("h_share_model start opt_with_lidar");
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         PointType &point_body  = feats_down_body->points[i];
//         PointType &point_world = feats_down_world->points[i];

//         /* transform to world frame */
//         V3D p_body(point_body.x, point_body.y, point_body.z);//p_I
//         V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);//p_w
//         point_world.x = p_global(0);
//         point_world.y = p_global(1);
//         point_world.z = p_global(2);
//         point_world.intensity = point_body.intensity;

//         // NUM_MATCH_POINTS   = 5
//         vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

//         auto &points_near = Nearest_Points[i];//最近邻点集

//         // 最近临平面搜索:构建abc, n(法向量)
//         if (ekfom_data.converge)
//         {
//             /** Find the closest surfaces in the map **/
//             // 在ikdtree的p_w附近寻找最近邻points_near
//             ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
//             //point_selected_surf点必须>=5,且pointSearchSqDis<=5为true
//             point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
//         }

//         if (!point_selected_surf[i]) continue;

//         VF(4) pabcd;
//         point_selected_surf[i] = false;
//         if (esti_plane(pabcd, points_near, 0.1f))
//         {
//             // pd2 = ax + by + cz + d = 0
//             float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
//             float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

//             if (s > 0.9)
//             {
//                 point_selected_surf[i] = true;
//                 normvec->points[i].x = pabcd(0);
//                 normvec->points[i].y = pabcd(1);
//                 normvec->points[i].z = pabcd(2);
//                 normvec->points[i].intensity = pd2;
//                 res_last[i] = abs(pd2);//找到了平面的abc, norm
//             }
//         }
//     }

//     effct_feat_num = 0;
//     // 找到有效的特征数量
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         if (point_selected_surf[i])
//         {
//             laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];//p_I
//             corr_normvect->points[effct_feat_num] = normvec->points[i];//abc+norm
//             total_residual += res_last[i];//|d|
//             effct_feat_num ++;
//         }
//     }
// //    ROS_WARN_STREAM("effct_feat_num " << effct_feat_num);

//     if (effct_feat_num < 1)
//     {
//         ekfom_data.valid = false;
//         ROS_WARN("No Effective Points! \n");
//         return;
//     }

//     //结束时刻的last
//     res_mean_last = total_residual / effct_feat_num;
//     match_time  += omp_get_wtime() - match_start;//matching time
//     double solve_start_  = omp_get_wtime();


//     /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
//     // 计算lidar的jacobian和观测方程
//     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //hx
//     ekfom_data.h.resize(effct_feat_num);//h

//     // set as zero before publish
//     A_p = Eigen::Matrix<double, 6, 6>::Zero();

//     // 构建每个effect_point的点面残差观测
//     for (int i = 0; i < effct_feat_num; i++)
//     {
//         const PointType &laser_p  = laserCloudOri->points[i];//p_l
//         V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
//         M3D point_be_crossmat;
//         point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
//         V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;//p_I
//         M3D point_crossmat;
//         point_crossmat<<SKEW_SYM_MATRX(point_this);

//         /*** get the normal vector of closest surface/corner ***/
//         const PointType &norm_p = corr_normvect->points[i];
//         V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

//         /*** calculate the Measuremnt Jacobian matrix H ***/
//         V3D C(s.rot.conjugate() *norm_vec);
//         V3D A(point_crossmat * C);

//         if (extrinsic_est_en)
//         {
//             V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
//             //H_i = [n, A, B, C]
//         }
//         else
//         {
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//         }

//         // calculate hissen matrix
//         Eigen::Matrix<double, 1, 6> H_i = Eigen::Matrix<double, 1, 6>::Zero();

//         H_i << VEC_FROM_ARRAY(A), norm_vec.x(), norm_vec.y(), norm_vec.z();
//         A_p += H_i.transpose() * H_i;//direct add A_p
//         /*** Measuremnt: distance to the closest surface/corner ***/
//         ekfom_data.h(i) = -norm_p.intensity;//点面残差 zj = d
//     }



//     solve_time += omp_get_wtime() - solve_start_;
// }
//  ######## h_share_simulation ######


// ============= //
// ############## h_share real robot #################//
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    // gnss的观测方程构建
    if (opt_with_gnss){
        // ROS_INFO("h_share_model start opt_with_gnss");
        ekfom_data.z = MatrixXd::Zero(3, 1);//z为名义观测
        ekfom_data.h_x = MatrixXd::Zero(3, 36);//hx为含delta_x和noise的观测方程
        ekfom_data.h.resize(3);
        ekfom_data.R = MatrixXd::Zero(3, 3);//观测误差
        ekfom_data.h_v = MatrixXd::Identity(3, 3);//观测noise
        // residual (estimate heading)
        V3D gnss_pos(Measures.gnss.front()->pose.pose.position.x, Measures.gnss.front()->pose.pose.position.y, Measures.gnss.front()->pose.pose.position.z);
        V3D res = s.offset_R_G_I * gnss_pos - s.pos;//误差量：观测 - 预测(gnss_pos为gnss在G系下的位置, 利用offset_R_G_I将其转移到I系在G系下的位置)
        ekfom_data.h(0) = res.x();
        ekfom_data.h(1) = res.y();
        ekfom_data.h(2) = res.z();
        // jacobian (estimate heading)
        ekfom_data.h_x.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity(); // d_dp
        M3D crossmat;
        crossmat << SKEW_SYM_MATRX(gnss_pos);
        auto P = kf.get_P();
        cout << "roll std " << sqrt(P(30,30)) << " pitch std" << sqrt(P(31, 31)) << " yaw std " << sqrt(P(32, 32)) << endl;
        // if yaw std converges do not estimate R_G_I
        if (sqrt(P(32, 32)) > 1e-4)
        {
            ekfom_data.h_x.block<3, 3>(0, 30) = -(s.offset_R_G_I * crossmat); // d_dR_G_I
            ROS_WARN("Estimate R_G_I !");
        }
        // covariance
        ekfom_data.R(0, 0) = 0.01 * Measures.gnss.front()->pose.covariance[0];
        ekfom_data.R(1, 1) = 0.01 * Measures.gnss.front()->pose.covariance[7];
        ekfom_data.R(2, 2) = Measures.gnss.front()->pose.covariance[14];
        // 当GNSS或者WHEEL的h和hx构建完后，直接return, 不进行lidar构建
        // 然后将opt_with_GNSS和opt_with_wheel置为false
        return;
    }

    // 轮速计观测方程构建
    // 当且仅当opt_with_wheel为true,即wheel时间戳在head, tail之间时，利用wheel来进行更新
    // 更新完opt_with_wheel置为false，不影响其他传感器的观测
    if (opt_with_wheel){
        // ROS_INFO("h_share_model start opt_with_wheel");
        ekfom_data.z = MatrixXd::Zero(3, 1);
        ekfom_data.h_x = MatrixXd::Zero(3, 36);
        ekfom_data.h.resize(3);
        ekfom_data.R = MatrixXd::Zero(3, 3);
        ekfom_data.h_v = MatrixXd::Identity(3, 3);
        // residual
        M3D angv_crossmat;
        V3D gyr = kf.get_input().gyro;//gyr
        V3D gyr_vec(gyr[0] - s.bg(0), gyr[1] - s.bg(1), gyr[2] - s.bg(2));//gyr_vec = gyr - bg
        angv_crossmat << SKEW_SYM_MATRX(gyr_vec);
        wheel_velocity = Measures.wheel.front()->twist.twist.linear.x;//前进速度
        V3D wheel_v_vec(wheel_velocity,0.0,0.0);

        // s.wheel_s 是一个尺度因子（scale factor），用于校正车轮速度的测量值。
        // ROS_WARN_STREAM("wheel_velocity " << wheel_velocity);
        V3D res = wheel_v_vec * s.wheel_s - s.offset_R_W_I.toRotationMatrix().transpose() * (s.rot.toRotationMatrix().transpose() * s.vel + angv_crossmat * s.offset_T_W_I);
        // ROS_WARN_STREAM("res " << res.transpose());
        ekfom_data.h(0) = res.x();
        ekfom_data.h(1) = res.y();
        ekfom_data.h(2) = res.z();
        // jacobian
        M3D rot_crossmat;
        V3D tmp_vel = s.rot.toRotationMatrix().transpose() * s.vel;
        rot_crossmat << SKEW_SYM_MATRX(tmp_vel); // 当前状态imu系下 点坐标反对称矩阵
        ekfom_data.h_x.block<3, 3>(0,3) = -s.offset_R_W_I.toRotationMatrix().transpose() * rot_crossmat; // diff w.r.t. rot
        ekfom_data.h_x.block<3, 3>(0,12) = -s.offset_R_W_I.toRotationMatrix().transpose() * s.rot.toRotationMatrix().transpose(); // diff w.r.t. vel
        M3D bg_crossmat;
        bg_crossmat << SKEW_SYM_MATRX(s.offset_T_W_I);
        ekfom_data.h_x.block<3, 3>(0,15) = -s.offset_R_W_I.toRotationMatrix().transpose() * bg_crossmat; // diff w.r.t. bg
        if (extrinsic_est_wheel){
            V3D tmp_vec = s.offset_R_W_I.toRotationMatrix().transpose() * (s.rot.toRotationMatrix().transpose() * s.vel + angv_crossmat * s.offset_T_W_I);
            M3D ex_rot_crossmat;
            ex_rot_crossmat << SKEW_SYM_MATRX(tmp_vec);
            ekfom_data.h_x.block<3, 3>(0,23) = -ex_rot_crossmat;
            ekfom_data.h_x.block<3, 3>(0,26) = -s.offset_R_W_I.toRotationMatrix().transpose() * angv_crossmat;
        }

        if (scale_est_wheel){
            ekfom_data.h_x.block<3, 1>(0,29) = wheel_v_vec;
        }

        // std::cout << "[ wheel ]: h_x: \n" << std::endl;
        // std::cout << ekfom_data.h_x << std::endl;

        // covariance
        Eigen::Matrix3d tmp_mat = s.offset_R_W_I.toRotationMatrix().transpose() * bg_crossmat;
        Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Identity();
        cov_mat(0, 0) = wheel_cov;
        if (gyr_vec.norm() > 0.3){
            cov_mat(1, 1) = wheel_velocity * gyr_vec.norm();
        }else{
            cov_mat(1, 1) = nhc_y_cov;
        }
        cov_mat(2, 2) = nhc_z_cov;
        cov_mat = cov_mat + tmp_mat * tmp_mat.transpose() * gyr_cov;
        ekfom_data.R = cov_mat;
        return;
    }

    // uwb观测方程构建

    if (opt_with_uwb){

        // p_mobile->getInnovation();

        // ROS_INFO("h_share model start opt_with_uwb");
        int k_uwb_size = p_mobile->m_ranges_to_anchors.size();
        // int k_uwb_size=current_uwb_data_.connected_nodes.size();//uwb个数
        // std::cout<<"received all uwb stations "<<k_uwb_size<<std::endl;
        ekfom_data.z = MatrixXd::Zero(k_uwb_size, 1);
        ekfom_data.h_x = MatrixXd::Zero(k_uwb_size, 36);
        ekfom_data.h.resize(k_uwb_size);
        ekfom_data.R = MatrixXd::Zero(k_uwb_size, k_uwb_size);
        ekfom_data.h_v = MatrixXd::Identity(k_uwb_size, k_uwb_size);

        state_point = kf.get_x();//得到当前状态,暂时不用定义
        if(0==k_uwb_size){
            std::cout<<"no workable uwb stationes"<<std::endl;
        }
        else
        {
            // 所有观测的size
            Eigen::MatrixXd Y(k_uwb_size,1);
            Eigen::MatrixXd Gt(k_uwb_size, 36);//  Dynamic matrix
            Eigen::MatrixXd R = MatrixXd::Zero(k_uwb_size, k_uwb_size);
            // H 矩阵
            for(int m=0;m<k_uwb_size;m++){
                for(int n=0;n<36;n++){
                    Gt(m,n)=0;
                }
            }

            uwb_station_marker.points.clear();
            //进行滤波更新

            // ROS_INFO("start uwb update sharemodel");
            std::map<int, common_msgs::UWB_FullRangeInfo>::iterator iter;
            int i = 0;
            for(iter=p_mobile->m_ranges_to_anchors.begin();iter!= p_mobile->m_ranges_to_anchors.end() && i < k_uwb_size;++iter, ++i)
            {
                int anchor_id = iter->first;
                common_msgs::UWB_FullRangeInfo range_info = iter->second;
                // 得到counterInc
                // p_mobile->readingCounterInc(anchor_id);

                geometry_msgs::PointStamped anchor_position;
                /*debug code*/
                // bool is_get_anchor = p_mobile->getAnchorPosition(anchor_id, anchor_position);
                // ROS_INFO("is_get_anchor = %s", is_get_anchor ? "true" : "false");
                // double anchor_range = p_mobile->getRangeMeterToAnchor(anchor_id);
                // ROS_INFO("anchor id: %d, get_from_anchor_range: %f", anchor_id, anchor_range);
                // ROS_INFO("anchor_position: %f, %f, %f", anchor_position.point.x, anchor_position.point.y, anchor_position.point.z);
                if(p_mobile->getAnchorPosition(anchor_id, anchor_position))
                {
                    // std::cout << "getAnchorPosition beg" << std::endl;
                    // int current_uwb_station_id = ndb_msg->neighbors.at(i).nodeId;
                    V3D tag_position;//p_GU
                    V3D position_I2G = state_point.pos;//p_GI
                    tag_position = state_point.pos + state_point.rot * state_point.offset_T_U_I;//p_GU = p_GI + RGI * p_IU
                    V3D anchor_pos_eigen = V3D(anchor_position.point.x
                                                                ,anchor_position.point.y
                                                                ,anchor_position.point.z);
                    // V3D temp_uwb_dis=tag_position-init_position_class.uwb_stations_position[current_uwb_station_id];
                    V3D temp_uwb_dis=tag_position - anchor_pos_eigen;//先验约束

                    //std::cout<<"uwb_stations_position "<<init_position_class.uwb_stations_position[current_uwb_station_id]<<std::endl;
                    //std::cout<<"tag_position "<<tag_position<<std::endl;
                    //std::cout<<"t emp_uwb_dis "<<temp_uwb_dis<<std::endl;
                    //////////////////////
                    geometry_msgs::Point p;
                    p.x = anchor_position.point.x;
                    p.y = anchor_position.point.y;
                    p.z = anchor_position.point.z;

                    uwb_station_marker.points.push_back(p);//发布marker

                    double sigma_r = double(range_info.precisionRangeErrEst) / 1000.0; // rangeErr
                    double sigma_a = kalman_sigma_a;
                    double r_meas = double(range_info.precisionRangeMm) / 1000.0; // RangeMm: RangeMeasurement (按照m来计算距离)
                    ROS_INFO("sigma_r: %f, sigma_a: %f, r_meas: %f", sigma_r, sigma_a, r_meas);
                    ////////////////////////
                    // temp_uwb_dis(2,0)=0;////////////////////
                    double r_pred=temp_uwb_dis.norm();

                    //std::cout<<"current_uwb_station_id "<<current_uwb_station_id<<std::endl;
                    //std::cout<<"uwb_observation_dis "<<observation_dis<<std::endl;
                    double delta_dis = r_meas - r_pred;
                    Y(i, 0)= delta_dis;//距离约束dist
                    // std::cout<<"delta_dis "<<delta_dis << std::endl;
                    // V3D error_state_delta_pos;
                    // error_state_delta_pos<<error_state_.x(0,0),error_state_.x(1,0),error_state_.x(2,0);//errror state
                    // V3D dis_vector= anchor_pos_eigen - error_state_delta_pos - tag_position;//range和nominalrange的差值
                    V3D dis_vector= anchor_pos_eigen - tag_position;//range和nominalrange的差值
                    // dis_vector(2,0)=0;/////////////////////////
                    M3D ex_U2I_crossmat;
                    ex_U2I_crossmat << SKEW_SYM_MATRX(state_point.offset_T_U_I);//UWB2IMU外参
                    Gt.block<1, 3>(i, 0) = dis_vector.transpose()*(1/dis_vector.norm());

                    // 去掉r_U对R_GI的约束,否则会扰乱系统的旋转，即旋转后点云也旋转了
                    // Gt.block<1, 3>(i, 3) = -(1/dis_vector.norm()) * dis_vector.transpose() * state_point.rot.toRotationMatrix() * ex_U2I_crossmat;
                    // std::cout<<"dis_vector.norm() = "<<dis_vector.norm()<<std::endl;
                    // std::cout << "getAnchorPosition end" << std::endl;

                    // 噪声矩阵
                    R(i, i) = std::pow(sigma_r, 2) * Q_scale;
                    // 将range设置给anchor
                    // p_mobile->m_last_range_time = range_info.header.stamp;
                    // p_mobile->successfulReadingCounterInc(anchor_id); // for statistics, to evaluate valid frequency
                }
            }

            uwb_station_marker_pub.publish(uwb_station_marker);

            Eigen::MatrixXd Ct(k_uwb_size, k_uwb_size);
            //init Ct(h_v)
            for(int m=0;m<k_uwb_size;m++){
                for(int n=0;n<k_uwb_size;n++){
                    if(m==n){
                        Ct(m,n)=1;
                    }
                    else{
                        Ct(m,n)=0;
                    }
                }
            }

            //std::cout<<"Ct "<<Ct<<std::endl;
            std::cout << "h:\n" << Y << std::endl;
            std::cout << "h_x.block<3,3>(0, 0): \n" << Gt.block<3,3>(0, 0) << std::endl;
            std::cout<<"R:\n"<<R<<std::endl;

            // update ekfom_data
            // ROS_INFO("update ekfom_data");
            ekfom_data.h = Y;
            ekfom_data.h_x = Gt;
            ekfom_data.R = R;//R != 0, otherwise,the system is going to crash.
            ekfom_data.h_v = Ct;//h_v is identity alike
        }

        return;
    }


    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    // 临近平面搜索和残差构建
    // 当执行update_iterated_dyn_share_modified时,然后将opt_with_GNSS和opt_with_wheel都为false，仅执行点面残差观测h,hx
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    // 构建feats_down_size个点的点面残差
    // ROS_INFO("h_share_model start opt_with_lidar");
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);//p_I
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);//p_w
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        // NUM_MATCH_POINTS   = 5
        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];//最近邻点集

        // 最近临平面搜索:构建abc, n(法向量)
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            // 在ikdtree的p_w附近寻找最近邻points_near
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            //point_selected_surf点必须>=5,且pointSearchSqDis<=5为true
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            // pd2 = ax + by + cz + d = 0
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);//找到了平面的abc, norm
            }
        }
    }

    effct_feat_num = 0;
    // 找到有效的特征数量
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];//p_I
            corr_normvect->points[effct_feat_num] = normvec->points[i];//abc+norm
            total_residual += res_last[i];//|d|
            effct_feat_num ++;
        }
    }
//    ROS_WARN_STREAM("effct_feat_num " << effct_feat_num);

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    //结束时刻的last
    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;//matching time
    double solve_start_  = omp_get_wtime();


    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    // 计算lidar的jacobian和观测方程
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //hx
    ekfom_data.h.resize(effct_feat_num);//h

    // set as zero before publish
    A_p = Eigen::Matrix<double, 6, 6>::Zero();

    // 构建每个effect_point的点面残差观测
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];//p_l
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;//p_I
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);

        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            //H_i = [n, A, B, C]
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        // calculate hissen matrix
        Eigen::Matrix<double, 1, 6> H_i = Eigen::Matrix<double, 1, 6>::Zero();

        H_i << VEC_FROM_ARRAY(A), norm_vec.x(), norm_vec.y(), norm_vec.z();
        A_p += H_i.transpose() * H_i;//direct add A_p
        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;//点面残差 zj = d
    }



    solve_time += omp_get_wtime() - solve_start_;
}
// ############## h_share real robot #################//


// uwb mobile
void createUWBMobile(const ros::NodeHandle& nh, const int mobile_id){
    // get anchor list from parameter server
    std::string param_key("uwb/anchor_list");
    if(!nh.getParam(param_key, m_anchor_list)){
        ROS_ERROR("Can't find anchor list param.");
        return;
    }

    // get anchor position, build anchor_map
    for(int i=0;i<m_anchor_list.size();++i){
        int anchor_id = m_anchor_list.at(i);
        // 该位置也得加uwb/xxx标号！否则无法构建m_anchor_map!
        param_key = std::string("uwb/anchor_") + num2str(anchor_id);
        std::vector<double> position;
        if(nh.getParam(param_key, position) && position.size()==3)
        {
            // has valid position
            // set anchor, variance set to constant 1
            boost::shared_ptr<UWB_Anchor_ESKF> p_anchor(
                        new UWB_Anchor_ESKF( anchor_id,
                                               position.at(0), position.at(1), position.at(2),
                                               1, 1, 1 ) );
            m_anchor_map.insert(std::pair<int, boost::shared_ptr<UWB_Anchor_ESKF> >(anchor_id, p_anchor) );
        }
        vec_anchor.push_back(V3D(position.at(0), position.at(1), position.at(2)));
    }

    // 求出节点间最大距离
    for (size_t i = 0; i < vec_anchor.size(); ++i) {
        for (size_t j = i + 1; j < vec_anchor.size(); ++j) {
            double distance = (vec_anchor[i] - vec_anchor[j]).norm();
            max_distance = std::max(max_distance, distance);
        }
    }


    ROS_WARN("max_distance of anchor_list: %f", max_distance);
    sleep(3);
    // create pointer
    p_mobile = boost::shared_ptr<UWB_Mobile_ESKF>(new UWB_Mobile_ESKF(mobile_id, m_anchor_map, nh) );
}

// publish observation
void publishObservableDirections(
    const Eigen::Matrix<double, 6, 6>& A,
    const Eigen::Quaterniond& q_G_I,
    const Eigen::Vector3d& p_G_I)
{
    // For visualization, plot only translation (not coupling between rotation
    // and translation)
    Eigen::Matrix<double, 3, 1> eigenvalues3;
    Eigen::Matrix<double, 3, 3> eigenvectors3;
    Eigen::Matrix<double, 3, 3> A_translation = A.block(3, 3, 3, 3);//下三角方阵
    doEigenDecomp3x3(A_translation, eigenvalues3, eigenvectors3);//3x3 Eigen分解

    // #####获得######
    //eigenValues3和eigenVectors3对应的是3个特征值和3个特征向量
    //获得R和t
    Eigen::Vector3d t_current = p_G_I;
    Eigen::Quaterniond q_current = q_G_I;

    //定义index
    Eigen::MatrixXf::Index maxRow_z, maxCol_z;

    /// @brief: 找到eigenVectors3矩阵的第3行(最小特征值的特征向量)中绝对值最大的元素，并返回这个元素的列索引到maxCol_z。
    /// @note: 我们是对一个行向量（1xN的矩阵）使用.maxCoeff()，因此实际上只有一个“行”和一个“列”。
    //maxRow_z会得到这个最大元素的“行”索引，它在这里应该是固定的（因为是第3行），而maxCol_z会得到这个最大元素的“列”索引
    //如果eigenVectors3.row(2) = [2, 3, 1], 则maxRow_z = 2， maxCol_z = 1
    (eigenvectors3.row(2))
        .cwiseAbs()  //它是对eigenvectors3的第3行中的每一个元素取绝对值。
        .maxCoeff(&maxRow_z, &maxCol_z); //寻找矩阵或向量中绝对值最大的元素，并返回这个最大值。同时，它接受两个指针参数，用于返回该最大元素在矩阵或向量中的位置。

    //定义directions
    visualization_msgs::MarkerArray observability_directions;
    geometry_msgs::Vector3 observability_vector;
    observability_vector.x = 0;
    observability_vector.y = 0;
    observability_vector.z = 0;

    //单位化的特征向量
    Eigen::Matrix<double, 3, 1> eigenvalues_normalized =
        eigenvalues3 / (eigenvalues3.maxCoeff());

    /** @brief
     * 1.这个循环遍历每个特征向量,对于每个特征向量，代码首先获取其对应的归一化特征值，然后提取特征向量的方向。
     * 2.接下来，它根据特征向量的第二个分量（Y轴分量）的正负性来调整特征向量的方向，以确保在可视化时的一致性。
     * 3.最后，如果当前处理的特征向量是对应于Z轴的特征向量（即列索引与`maxCol_z`相等），
     * 它会根据特征向量的第三个分量（Z轴分量）的正负性来调整特征向量的方向，确保Z轴方向是向上的。
     */
    for (int i = 0; i < eigenvalues3.rows(); i++)
    {
        double eigenvalue_normalized = eigenvalues_normalized(i);
        //取出这个特征值对应的特征向量
        Eigen::Vector3d direction(
            eigenvectors3(0, i),
            eigenvectors3(1, i),
            eigenvectors3(2, i)); // LAST three coordinates = translation coordinates

        //根据该特征向量Y的正负性来调整特征向量的方向,确保Y朝左
        int sign = (direction(1) > 0) ? 1 : -1;
        direction = sign * direction; // To keep consistency when plotting (v and
                                        // -v are two valids eigenvectors)

        // Ensure Eigenvector z is poiting up
        // Eigen::MatrixXf::Index maxRow, maxCol;
        // eigenvalues_normalized.maxCoeff(&maxRow, &maxCol); //Find which one is

        //若处理的是z轴特征向量，根据特征向量的第三个分量（Z轴分量）的正负性来调整特征向量的方向，确保Z轴方向是向上的
        if (i == maxCol_z)
        {
            int sign_z = (direction(2) > 0) ? 1 : -1;
            direction = sign_z * direction;
        }

        visualization_msgs::Marker marker;
        // marker.header = pclptr_normals->header;
        marker.header.frame_id = "camera_init";//header调整为camera_init
        marker.id = i + 5000;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        //定义起点和终点
        geometry_msgs::Point point_init, point_end;
        //让初始位置为base
        point_init.x = t_current[0];
        point_init.y = t_current[1];
        point_init.z = t_current[2];
        marker.points.push_back(point_init);

        point_end.x = point_init.x + eigenvalue_normalized * direction(0) / 0.16;
        point_end.y = point_init.y + eigenvalue_normalized * direction(1) / 0.16;
        point_end.z = point_init.z + eigenvalue_normalized * direction(2) / 0.16;
        marker.points.push_back(point_end);
        marker.scale.x = 0.13;
        marker.scale.y = 0.25;
        marker.scale.z = 0;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;
        marker.header.stamp = ros::Time::now();

        observability_directions.markers.push_back(marker);

        observability_vector.x += eigenvalue_normalized * direction(0);
        observability_vector.y += eigenvalue_normalized * direction(1);
        observability_vector.z += eigenvalue_normalized * direction(2);

        pub_eigen_marker.publish(observability_directions);
        pub_observability_vector.publish(observability_vector);
    }
}

// 绘制为椭球
void pubPositionCovarianceWorldCurrNWU(const geometry_msgs::Pose& pose, const Eigen::Matrix3d& covariance)
{
    common_msgs::PositionWithCovarianceStamped positionwithcovartiancestamped;
    positionwithcovartiancestamped.position = pose.position;
    positionwithcovartiancestamped.covariance[0] = covariance(0,0);
    positionwithcovartiancestamped.covariance[1] = covariance(0,1);
    positionwithcovartiancestamped.covariance[2] = covariance(0,2);
    positionwithcovartiancestamped.covariance[3] = covariance(1,0);
    positionwithcovartiancestamped.covariance[4] = covariance(1,1);
    positionwithcovartiancestamped.covariance[5] = covariance(1,2);
    positionwithcovartiancestamped.covariance[6] = covariance(2,0);
    positionwithcovartiancestamped.covariance[7] = covariance(2,1);
    positionwithcovartiancestamped.covariance[8] = covariance(2,2);

    positionwithcovartiancestamped.header.stamp = ros::Time::now();
    positionwithcovartiancestamped.header.frame_id = "camera_init";
    pubPositionWithCovariance.publish(positionwithcovartiancestamped);

    Eigen::Vector3f kp_mean(positionwithcovartiancestamped.position.x, positionwithcovartiancestamped.position.y, positionwithcovartiancestamped.position.z);
    Eigen::Matrix3f kp_cov;
    kp_cov <<
        covariance(0,0), covariance(0,1), covariance(0,2),
        covariance(1,0), covariance(1,1), covariance(1,2),
        covariance(2,0), covariance(2,1), covariance(2,2);

    cv::Mat m(3, 1, CV_64F);
    for (int j = 0; j < 3; ++j)
        m.at<double>(j, 0) = kp_mean(j, 0);

    cv::Mat cov(3, 3, CV_64F);
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i)
            cov.at<double>(j, i) = kp_cov(j, i);

    cv::Mat evl(1, 3, CV_64F);
    cv::Mat evt(3, 3, CV_64F);
    cv::eigen(cov, evl, evt);

    // double scale = 10.0;
    tf::Vector3 evt_tf(evt.at<double>(0,0), evt.at<double>(0,1), evt.at<double>(0,2));

    visualization_msgs::Marker marker;
    marker.header.stamp = positionwithcovartiancestamped.header.stamp;
    marker.header.frame_id = "camera_init";
    marker.type = visualization_msgs::Marker::SPHERE;

    // yello
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    marker.color.a = 0.7;

    // purge
    // marker.color.r = 0.5;
    // marker.color.g = 0.0;
    // marker.color.b = 0.5;
    // marker.color.a = 0.6;
    marker.pose.position.x = kp_mean.x();//设置位置
    marker.pose.position.y = kp_mean.y();
    marker.pose.position.z = kp_mean.z();
    marker.scale.x = evt_tf.getX() * position_cov_marker_scale;//设置大小
    marker.scale.y = evt_tf.getY() * position_cov_marker_scale;
    marker.scale.z = evt_tf.getZ() * position_cov_marker_scale;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "covariances_uwb_wheel";
    marker.id = 0;
    marker.lifetime = ros::Duration();

    pubUWBPositionMarker.publish(marker);
}



void runtime_pos_log_output()
{
    // // Average computation time
    // ROS_INFO("comp_times");
    double avg_comp_time =
    std::accumulate(comp_times.begin(), comp_times.end(), 0.0) / comp_times.size();

    // Average sensor rates
    int win_size = 100;
    double avg_imu_rate;
    double avg_lidar_rate;
    if (imu_rates.size() < win_size) {
        avg_imu_rate =
            std::accumulate(imu_rates.begin(), imu_rates.end(), 0.0) / imu_rates.size();
    } else {
        avg_imu_rate =
            std::accumulate(imu_rates.end()-win_size, imu_rates.end(), 0.0) / win_size;
    }
    if (lidar_rates.size() < win_size) {
        avg_lidar_rate =
            std::accumulate(lidar_rates.begin(), lidar_rates.end(), 0.0) / lidar_rates.size();
    } else {
        avg_lidar_rate =
            std::accumulate(lidar_rates.end()-win_size, lidar_rates.end(), 0.0) / win_size;
    }

    // // RAM Usage
    // ROS_INFO("RAM Usage");
    double vm_usage = 0.0;
    double resident_set = 0.0;
    std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
    std::string pid, comm, state2, ppid, pgrp, session, tty_nr;
    std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    std::string utime, stime, cutime, cstime, priority, nice;
    std::string num_threads, itrealvalue, starttime;
    unsigned long vsize;
    long rss;
    stat_stream >> pid >> comm >> state2 >> ppid >> pgrp >> session >> tty_nr
            >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
            >> utime >> stime >> cutime >> cstime >> priority >> nice
            >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
    stat_stream.close();
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;

    // CPU Usage
    // ROS_INFO("CPU Usage");
    struct tms timeSample;
    clock_t now;
    double cpu_percent;
    now = times(&timeSample);
    if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
        timeSample.tms_utime < lastUserCPU) {
        cpu_percent = -1.0;
    } else {
        cpu_percent = (timeSample.tms_stime - lastSysCPU) + (timeSample.tms_utime - lastUserCPU);
        cpu_percent /= (now - lastCPU);
        cpu_percent /= numProcessors;
        cpu_percent *= 100.;
    }
    lastCPU = now;
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;
    cpu_percents.push_back(cpu_percent);
    double avg_cpu_usage =
         std::accumulate(cpu_percents.begin(), cpu_percents.end(), 0.0) / cpu_percents.size();

    printf("\033[2J\033[1;1H");
    //直接用eskf_和last_imu_可以得到当前状态量 和 积分得到当前状态量的imu状态
    state_point = kf.get_x();
    // Eigen::Vector3d pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    double delta_t =  Measures.lidar_beg_time - first_lidar_time;
    Eigen::Vector3d position = state_point.pos;
    Eigen::Vector3d velocity = state_point.vel;
    Eigen::Vector3d euler_cur = Log(state_point.rot.toRotationMatrix());
    Eigen::Vector3d acc_bias = state_point.ba;
    Eigen::Vector3d gyro_bias = state_point.bg;
    S2 gravity = state_point.grav;

    std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
    std::cout << "|              CoalMine-LiDAR-Inerital-UWB-Wheel-Odometry "  << "          |"
            << std::endl;
    std::cout << "+-------------------------------------------------------------------+" << std::endl;

    std::time_t curr_time = scan_stamp; //scan_stamp为bag的时间：Nov 15 2023
    std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
    std::cout << "| " << std::left << asc_time;

    std::cout << std::right << std::setfill(' ') << std::setw(42)
    << "Elapsed Time: " + to_string_with_precision(delta_t, 2) + " seconds "
    << "|" << std::endl;

    std::cout << "| " << std::left << "  ";
    std::cout << std::right << std::setfill(' ') << std::setw(32)
    << "Index Frame: " + to_string_with_precision(index_frame, 2) << std::setfill(' ') << std::setw(33)
    << " |" << std::endl;

    if ( !cpu_type.empty() ) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << cpu_type + " x " + std::to_string(numProcessors)
        << "|" << std::endl;
    }

    if ( p_pre->lidar_type == AVIA) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Sensor Rates: AVIA @ " + to_string_with_precision(avg_lidar_rate, 2)
                                        + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
        << "|" << std::endl;
    } else if ( p_pre->lidar_type == VELO16) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
                                        + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
        << "|" << std::endl;
    } else if ( p_pre->lidar_type == OUST64) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Sensor Rates: OUST64 @ " + to_string_with_precision(avg_lidar_rate, 2)
                                + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
        << "|" << std::endl;
    } else if ( p_pre->lidar_type == RS128) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Sensor Rates: ROBOSENSE128 @ " + to_string_with_precision(avg_lidar_rate, 2)
                                + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
        << "|" << std::endl;
    } else if ( p_pre->lidar_type == HESAI32) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Sensor Rates: PANDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
        << "|" << std::endl;
    }
    else {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
        << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
        << "|" << std::endl;
    }

    // state
    std::cout << "|===================================================================|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Position     {W}  [xyz] :: " + to_string_with_precision(position[0], 4) + " "
                                + to_string_with_precision(position[1], 4) + " "
                                + to_string_with_precision(position[2], 4)
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Orientation  {W}  [rpy] :: " + to_string_with_precision(euler_cur[0], 4) + " "
                                + to_string_with_precision(euler_cur[1], 4) + " "
                                + to_string_with_precision(euler_cur[2], 4)
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Lin Velocity {B}  [xyz] :: " + to_string_with_precision(velocity[0], 4) + " "
                                + to_string_with_precision(velocity[1], 4) + " "
                                + to_string_with_precision(velocity[2], 4)
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(last_imu_->linear_acceleration.x, 4) + " "
                                + to_string_with_precision(last_imu_->linear_acceleration.y, 4) + " "
                                + to_string_with_precision(last_imu_->linear_acceleration.z, 4)
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Lin Accel    {B}  [xyz] :: " + to_string_with_precision(last_imu_->angular_velocity.x, 4) + " "
                                + to_string_with_precision(last_imu_->angular_velocity.y, 4) + " "
                                + to_string_with_precision(last_imu_->angular_velocity.z, 4)
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Accel Bias        [xyz] :: " + to_string_with_precision(acc_bias[0], 8) + " "
                                + to_string_with_precision(acc_bias[1], 8) + " "
                                + to_string_with_precision(acc_bias[2], 8)
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Gyro Bias         [xyz] :: " + to_string_with_precision(gyro_bias[0], 8) + " "
                                + to_string_with_precision(gyro_bias[1], 8) + " "
                                + to_string_with_precision(gyro_bias[2], 8)
    << "|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Gravity           [xyz] :: " + to_string_with_precision(gravity[0], 8) + " "
                                + to_string_with_precision(gravity[1], 8) + " "
                                + to_string_with_precision(gravity[2], 8)
    << "|" << std::endl;

    // distance
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance Traveled  :: " + to_string_with_precision(length_traversed, 4) + " meters"
    << "|" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance to Origin :: "
        + to_string_with_precision( sqrt(pow(position[0] - origin[0],2) +
                                        pow(position[1] - origin[1],2) +
                                        pow(position[2] - origin[2],2)), 4) + " meters"
    << "|" << std::endl;

    std::cout << "|                                                                   |" << std::endl;

    // computation time
    std::cout << std::right << std::setprecision(2) << std::fixed;
    std::cout << "| Computation Time :: "
    << std::setfill(' ') << std::setw(6) << comp_times.back()*1000. << " ms    // Avg: "
    << std::setw(6) << avg_comp_time*1000. << " / Max: "
    << std::setw(6) << *std::max_element(comp_times.begin(), comp_times.end())*1000.
    << "     |" << std::endl;
    std::cout << "| Cores Utilized   :: "
    << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * numProcessors << " cores // Avg: "
    << std::setw(6) << (avg_cpu_usage/100.) * numProcessors << " / Max: "
    << std::setw(6) << (*std::max_element(cpu_percents.begin(), cpu_percents.end()) / 100.)
                        * numProcessors
    << "     |" << std::endl;
    std::cout << "| CPU Load         :: "
    << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
    << std::setw(6) << avg_cpu_usage << " / Max: "
    << std::setw(6) << *std::max_element(cpu_percents.begin(), cpu_percents.end())
    << "     |" << std::endl;
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
    << "|" << std::endl;

    std::cout << std::right << std::setprecision(2) << std::fixed;
    std::cout << "+-------------------------------------------------------------------+" << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<std::string>("save_directory", saveDirectory, "/");
    nh.param<std::string>("common/odom_link", odom_link, "camera_init");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<string>("common/wheel_topic", wheel_topic,"/chinook/husky_velocity_controller/odom");
    nh.param<string>("common/ground_truth_topic", ground_truth_topic, "/ground_truth/state");//gt
    nh.param<string>("common/sequence", scene_name, "compus_road_day");

    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<bool>("common/ros_driver_fix_en", ros_driver_fix_en, false);
    nh.param<bool>("common/use_wheel", USE_WHEEL, false);
    cout << "use_wheel " << int(USE_WHEEL) << endl;
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);

    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);

    nh.param<double>("mapping/laser_point_cov",laser_point_cov, 0.001);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("wheel/wheel_cov",wheel_cov,0.01);
    nh.param<double>("wheel/nhc_y_cov",nhc_y_cov,0.01);
    nh.param<double>("wheel/nhc_z_cov",nhc_z_cov,0.01);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<string>("pcd_save/pcd_save_dir", pcd_save_dir, "~/Download/");
    nh.param<string>("pcd_save/pcd_name", pcd_name, "fastlio");
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    // wheel
    nh.param<bool>("wheel/extrinsic_est_wheel", extrinsic_est_wheel, true);
    nh.param<bool>("wheel/scale_est_wheel", scale_est_wheel, true);
    nh.param<vector<double>>("wheel/extrinsic_T", extrinT_wheel, vector<double>());
    nh.param<vector<double>>("wheel/extrinsic_R", extrinR_wheel, vector<double>());
    nh.param<vector<double>>("wheel/wheel_scale", wheel_scale, vector<double>());

    //// gnss
    nh.param<string>("common/gnss_topic", gnss_topic,"/gps/fix");
    nh.param<bool>("common/use_gnss", USE_GNSS, false);
    cout << "use_gnss " << int(USE_GNSS) << endl;
    nh.param<vector<double>>("mapping/extrinR_Gnss2IMU", extrinR_Gnss2IMU, vector<double>());
    nh.param<vector<double>>("mapping/extrinT_Gnss2IMU", extrinT_Gnss2IMU, vector<double>());
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";


    // uwb
    nh.param<string>("common/uwb_topic", uwb_topic, "/time_domain/NDB");
    nh.param<bool>("common/use_uwb", USE_UWB, false);
    cout << "use_uwb " << int(USE_UWB) << endl;
    nh.param<double>("uwb/slam_fps", m_slam_fps, 0);
    nh.param<double>("uwb/td", m_td, 0.3);//每一帧的uwb时间间隔
    nh.param<int>("uwb/mobile_id", m_mobile_id, 301);
    nh.param<std::vector<double>>("uwb/extrinsic_T", extrinT_UWB2IMU, std::vector<double>());
    nh.param<std::vector<int>>("uwb/anchor_list", m_anchor_list, std::vector<int>());
    nh.param<bool>("uwb/is_initialize_with_ceres", m_is_initialize_with_ceres, true);
    nh.param<bool>("uwb/is_runtime_output", is_runtime_output, true);
    nh.param<bool>("uwb/is_pub_position_covariance", is_pub_position_covariance, true);
          nh.param<double>("uwb/position_cov_marker_scale", position_cov_marker_scale, 1.0);//publish observation vector
    // nh.param<double>("uwb/uwb_noise", uwb_noise_, 0.1);//关于noise 暂时不用这个噪声
    // nh.param<double>("uwb/trajectory_noise", trajectory_noise_, 0.2);
    // nh.param<double>("uwb/non_hol_noise_z", non_hol_noise_z_, 0.05);
    nh.param<int>("uwb/N_distance", N_distance, 3);
    kNumAnchor_ = static_cast<int>(m_anchor_list.size()); // 使用static_cast确保类型安全
    // --- uwb noise related ------ //
    nh.param<double>("uwb/z_damping_factor", z_damping_factor, 0.1);//关于noise
    nh.param<double>("uwb/Q_scale", Q_scale, 10);
    nh.param<double>("uwb/R_scale", R_scale, 30);
    nh.param<double>("uwb/tao_acc_sqrt", tao_acc_sqrt, 0.4);
    nh.param<double>("uwb/tao_bias_sqrt", tao_bias_sqrt, 0.001);
    nh.param<double>("uwb/initialization_innovation_threshold", initialization_innovation_threshold, 2.0);
    nh.param<double>("uwb/normal_innovation_threshold", normal_innovation_threshold, 2.0);
    nh.param<double>("uwb/innovation_threshold_inc_on_succ", innovation_threshold_inc_on_succ, -0.2);
    nh.param<double>("uwb/innovation_threshold_inc_on_fail", innovation_threshold_inc_on_fail, 0.6);
    nh.param<double>("uwb/kalman_sigma_a", kalman_sigma_a, 0.125);
    // --- uwb noise related ------ //

    /*-------------configuration-----------------*    ROS_INFO("fastlio2-->started");


    /*--------------configuration----------------*/

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    // ROS_INFO("good1");
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);
    _featsArray.reset(new PointCloudXYZI());

    /***state output***/
    index_frame = 1;//第0帧初始化
    length_traversed = 0;
    origin = Eigen::Vector3f(0., 0., 0.);

    // CPU Specs
    char CPUBrandString[0x40];
    memset(CPUBrandString, 0, sizeof(CPUBrandString));

    cpu_type = "";

    #ifdef HAS_CPUID
    unsigned int CPUInfo[4] = {0,0,0,0};
    __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    unsigned int nExIds = CPUInfo[0];
    for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
        memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
        memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
        memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
    }
    cpu_type = CPUBrandString;
    boost::trim(cpu_type);
    #endif

    FILE* file;
    struct tms timeSample;
    char line[128];

    lastCPU = times(&timeSample);
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    numProcessors = 0;
    while(fgets(line, 128, file) != nullptr) {
        if (strncmp(line, "processor", 9) == 0) numProcessors++;
    }
    fclose(file);//read CPU time
    // output state initialization
    /***state output***/

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // 设置wheel外参
    Wheel_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT_wheel);
    Wheel_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR_wheel);
    p_imu->set_wheel_extrinsic(Wheel_T_wrt_IMU, Wheel_R_wrt_IMU);
    p_imu->set_wheel_nhc_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    wheel_s = V1D(wheel_scale[0]);
    cout << "[ wheel scale ]: " << wheel_scale[0] << endl;
    p_imu->set_wheel_scale(wheel_s);

    // 设置gnss外参数
    Gnss_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT_Gnss2IMU);
    Gnss_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR_Gnss2IMU);

    /*uwb param*/
    // uwb到IMU外参
    UWB_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT_UWB2IMU);
    p_imu->set_uwb_extrinsic(UWB_T_wrt_IMU);//uwb外参
    p_imu->set_uwb_nhc_cov(V3D(uwb_noise_, uwb_noise_, uwb_noise_));//uwb_noise
    p_imu->set_uwb_N_distance(N_distance);

    if(m_slam_fps<20){
        m_is_fix_fps = false;
    } else {
        m_is_fix_fps = true;
    }
    ROS_INFO("mobile_id: %d", m_mobile_id);
    // create UWB_Mobile_ESKF
    if( nh.getParam(std::string("/uav_id"), m_mobile_id ) ) {
        ROS_INFO("Mobile ID (/uav_id): %d", m_mobile_id);
    } else if( nh.getParam(std::string("uwb/mobile_id"), m_mobile_id ) ){
        ROS_INFO("Mobile ID (yaml): %d", m_mobile_id);
    } else {
        ROS_ERROR("Mobile ID not found!");
        return 0;
    }

    createUWBMobile(nh, m_mobile_id);
    ROS_INFO("UWB_Mobile_ESKF created.");
    p_imu->set_uwb_max_distance(max_distance);//设置UWB最大距离
    // uwb station part
    uwb_station_marker.header.frame_id = "camera_init";
    uwb_station_marker.header.stamp = ros::Time::now();
    uwb_station_marker.ns = "uwb_shapes";
    uwb_station_marker.id = 0;
    uwb_station_marker.type =visualization_msgs::Marker::SPHERE_LIST;//球序列;
    uwb_station_marker.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    uwb_station_marker.scale.x = 0.5;
    uwb_station_marker.scale.y = 0.5;
    uwb_station_marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    uwb_station_marker.color.r = 0.0f;
    uwb_station_marker.color.g = 1.0f;
    uwb_station_marker.color.b = 0.0f;
    uwb_station_marker.color.a = 1.0;

    //初始化所有传感器观测方程zj和观测矩阵Hj(必须为36,维度和状态量一致，否则程序会挂掉)
    // double epsi[23] = {0.001};
    // fill(epsi, epsi+23, 0.001);
    double epsi[36] = {0.001};
    fill(epsi, epsi + 36, 0.001);//limit = epsi[36] = {0.001};
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);

    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_wheel;
    if (USE_WHEEL)
        sub_wheel = nh.subscribe(wheel_topic, 200000, wheel_cbk);
    ros::Subscriber sub_gnss;
    if (USE_GNSS)
        sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk);
    ros::Subscriber sub_uwb;
    if (USE_UWB)
        // ROS_INFO("uwb_cbk");
        sub_uwb =  nh.subscribe(uwb_topic, 200000, uwb_cbk);

    // subscribe gt
    // ros::Subscriber sub_ground_truth = nh.subscribe(ground_truth_topic, 200000, ground_truth_cbk);

    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_ori", 100000);

    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body_ori", 100000);

    // ros::Publisher pubLaserCloudFull_body_raw =
    //         nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body_raw",
    //                                                100000);

    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected_ori", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map_ori", 100000);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/Odometry_ori", 100000);

    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
            ("/path_ori", 100000);
    ros::Publisher pubGnssPath = nh.advertise<nav_msgs::Path>("/gnss_path", 100000);
    pubGnssCov = nh.advertise<geometry_msgs::Vector3>("/gnss_cov", 100000);

    // publish gt path
    ros::Publisher pub_ground_truth_path = nh.advertise<nav_msgs::Path>
            ("/path_ground_truth_ori", 100000);
    /*publish uwb related*/
    // ROS_INFO("pubUWBPositionMarker");//这里是marker而非markerArray
    pubUWBPositionMarker = nh.advertise<visualization_msgs::Marker>("/uwb_position_marker_lio", 1000);//发布cov array
    // ROS_INFO("pubPositionWithCovariance");
    pubPositionWithCovariance = nh.advertise<common_msgs::PositionWithCovarianceStamped>("/position_with_covariance_lio",10);//发布positionCov
    // ROS_INFO("uwb_station_marker_pub");
    uwb_station_marker_pub = nh.advertise<visualization_msgs::Marker>("/live_uwb_station", 100);//发布station


    // 发布direction marker
    pub_eigen_marker =  nh.advertise<visualization_msgs::MarkerArray>("/eigen_vector", 10);
    pub_observability_vector = nh.advertise<geometry_msgs::Vector3>("/observability_vector", 10);


    // Debug statements and publish custom DLIO message
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    std::vector<int> time_usage_in_ms_;
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();//问题出在rosSpinOnce，程序挂了
        if(sync_packages(Measures))
        {
            index_frame++;
            double then = ros::Time::now().toSec();//时间间隔
            auto avg_time_t1 = std::chrono::steady_clock::now();
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }
            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            // ROS_INFO("main-1");
            // p_imu->Process(Measures, kf, feats_undistort, feats_undistort_full, p_mobile);
            p_imu->Process(Measures, kf, feats_undistort, p_mobile);

            state_point = kf.get_x();//得到该帧末尾的状态

            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;// p_GL = R_GI * p_IL,lidar在G下的坐标

            // ROS_INFO("feats_undistort-before");
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            // ROS_INFO("main-2");
            // ROS_INFO("feats_undistort-after");

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();


            // ROS_INFO("main-3");
            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            // ROS_INFO("main-4");
            if (0) {
                // effct_feat_num represent for map number
                cout << "[ mapping ]: In num: " << feats_undistort->points.size() << " downsamp " << feats_down_size
                     << " Map num: " << featsFromMapNum << " effectnum: " << effct_feat_num << endl;
            }

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // ROS_INFO("main-4");
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            // ROS_INFO("main-5");
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            // ROS_INFO("main-6");
            // laser_point_cov对精度影响至关重要，cov增加，状态X和协方差P增大，lidar精度降低，其他传感器精度提高；
            // cov减少，状态X和协方差P减少，lidar精度增加，其他传感器精度降低
            kf.update_iterated_dyn_share_modified(laser_point_cov, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            // ROS_INFO("main-7");
            double t_update_end = omp_get_wtime();

            // ROS_INFO("main-8");
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            // ROS_INFO("main-9");
            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            // ROS_INFO("main-10");
            /******* Publish points *******/
            // ROS_INFO("main-11");
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            publish_frame_body(pubLaserCloudFull_body);//发布body系下点云
            // ROS_INFO("main-12");
            // pub raw point cloud
            // publish_frame_body_raw(pubLaserCloudFull_body_raw);//发布body_raw
            publish_effect_world(pubLaserCloudEffect);//cloudEffect
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            // ROS_INFO("main-13");
            // 发布position cov
            if (is_pub_position_covariance)
            {
                // ROS_INFO("start pub position_covariance");
                Eigen::Matrix<double, 36, 36> covariance_temp = kf.get_P();//得到当前的cov

                std::cout << "covariance_temp_p_theta: " << std::endl;
                Eigen::Matrix<double, 6, 6> cov_p_theta = covariance_temp. template block<6, 6>(0, 0);
                std::cout << cov_p_theta << std::endl;
                m_position.position.x = state_point.pos[0];//得到当前的position
                m_position.position.y = state_point.pos[1];
                m_position.position.z = state_point.pos[2];
                Eigen::MatrixXd cov_kAnchor_position(3, 3); // 动态矩阵，不使用模板参数指定行列数
                // cov_kAnchor_position = covariance_temp.block(0, 0, kNumAnchor_, 3);//只能使用这个
                cov_kAnchor_position = covariance_temp.block(0, 0, 3, 3);//只能使用这个, p分量
                m_position_covariance = cov_kAnchor_position;
                // m_position_covariance = cov_kAnchor_position.transpose() * cov_kAnchor_position;
                // std::cout << "m_position_covariance: " << std::endl;
                // std::cout << m_position_covariance << std::endl;
                // 这行代码不能有！
                // m_position_covariance(0,0)=covariance_temp(0,0);
                // m_position_covariance(0,1)=covariance_temp(0,2);
                // m_position_covariance(0,2)=covariance_temp(0,4);
                // m_position_covariance(1,0)=covariance_temp(2,0);
                // m_position_covariance(1,1)=covariance_temp(2,2);
                // m_position_covariance(1,2)=covariance_temp(2,4);
                // m_position_covariance(2,0)=covariance_temp(4,0);
                // m_position_covariance(2,1)=covariance_temp(4,2);
                // m_position_covariance(2,2)=covariance_temp(4,4);
                pubPositionCovarianceWorldCurrNWU(m_position, m_position_covariance);
            }

            // ROS_INFO("main-14");
            // 发布position cov
            if (is_pub_observation_vector)
            {
                // T_GI
                publishObservableDirections(A_p, state_point.rot, state_point.pos);
            }
            /*estimate the dist*/
            if(index_frame%8==0)
            {
                static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                Eigen::Vector3d t = state_point.pos;
                static double dist = 0;
                dist += (t - last_t).norm();
                last_t = t;
                length_traversed = dist;
            }

            // ROS_INFO("main-15");
            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                // printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                if (USE_WHEEL)
                {
                    if (extrinsic_est_wheel)
                    {
                        ext_euler = SO3ToEuler(state_point.offset_R_W_I);
                        cout << setw(20) << "R_W_I " << ext_euler.transpose() << " " << "T_W_I "
                             << state_point.offset_T_W_I.transpose() << " " << endl;
                    }
                    if (scale_est_wheel)
                        cout << "wheel scale " << state_point.wheel_s << endl;
                }
                if (USE_GNSS)
                {
//                    ext_euler = SO3ToEuler(state_point.offset_R_G_I);
//                    cout << setw(20) << "R_G_I " << ext_euler.transpose() << endl;
                    GNSS_Heading = state_point.offset_R_G_I.toRotationMatrix();
                }
                dump_lio_state_to_log(fp);
            }

            // ROS_INFO("main-16");
            // runtime_output一定在runtime_pos_log_output前面，否则失败
            comp_times.push_back(ros::Time::now().toSec() - then);

            // ROS_INFO("main-17");
            /*Debug for uwb(这里极有可能会降低建图的精度和效果，因为在处理每一帧时，还额外增加)*/
            //打印消息(创建一个线程分离出去)
            // 方案1：直接output
            if (is_runtime_output)
            {
                runtime_pos_log_output();//原本的状态
                std::thread debug_thread = std::thread(&runtime_pos_log_output);
                debug_thread.detach();
            }
            // ROS_INFO("main-18");
            // 方案2：维持不变
            // if (is_runtime_output)
            // {
                // runtime_pos_log_output();
            // }
            auto avg_time_t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(avg_time_t2 - avg_time_t1).count() * 1000;
            time_usage_in_ms_.emplace_back(time_used);

            // ROS_INFO("main-19");
        }

        status = ros::ok();
        rate.sleep();
    }

    // ***************calculated average time************* //
    {
        double average_time = std::accumulate(time_usage_in_ms_.begin(), time_usage_in_ms_.end(), 0.0) /
                                double(time_usage_in_ms_.size());

        std::cout << "\x1b[0;32m" << "> [" << "FAST-LIO2-MSF-ORI" << " ] average time usage: "
                  << average_time
                  << " ms, called times: " << time_usage_in_ms_.size() << "\x1b[0m" << std::endl;
        // output the time usage log
        std::string time_usage_log_path = root_dir + "Log/time_usage_log.csv";
        std::ofstream time_usage_log_file(time_usage_log_path, std::ios::out);
        if (time_usage_log_file.is_open()) {

            // frame和每一帧的时间
            // time_usage_log_file << "frame_index,time_usage_in_ms\n";
            // for (size_t i = 0; i < time_usage_in_ms_.size(); ++i) {
            //     time_usage_log_file << i + 1 << "," << time_usage_in_ms_[i] << "\n";
            // }

            // average time
            time_usage_log_file << "average," << average_time << "ms, called times: " << time_usage_in_ms_.size() << "\n";

            time_usage_log_file.close();
            std::cout << "\x1b[0;32m"
                      << "> [FAST-LIO2-MSF-ORI] Time usage log saved to: " << time_usage_log_path
                      << "\x1b[0m" << std::endl;
        } else {
            std::cerr << "\x1b[0;31m"
                      << "> [FAST-LIO2-MSF-ORI] Failed to open time usage log file: " << time_usage_log_path
                      << "\x1b[0m" << std::endl;
        }
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        // string file_name = string("scans.pcd");
        // string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        string all_points_dir(pcd_save_dir + pcd_name + string("-scans.pcd"));
        pcl::PCDWriter pcd_writer;

        // cout << "current scan saved to /PCD/" << file_name<<endl;
        ROS_WARN("current scan saved to /PCD/: %s", all_points_dir.c_str());
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();


    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
