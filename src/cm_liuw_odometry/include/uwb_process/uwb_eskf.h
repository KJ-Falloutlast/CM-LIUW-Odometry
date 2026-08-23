#ifndef _UWB_ESKF_H_
#define _UWB_ESKF_H_

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_set>

#include <boost/shared_ptr.hpp>
#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <utility> //pair
#include <fstream>
#include <iomanip> //setw
#include <string.h>
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "common_msgs/UWB_FullRangeInfo.h"
#include "common_msgs/UWB_FullNeighborDatabase.h"
#include "common_msgs/UWB_DataInfo.h"
#include "common_msgs/UWB_SendData.h"
#include "common_msgs/UWB_EchoedRangeInfo.h"
#include "common_msgs/NavigationState.h"

#include "common_msgs/GPSVelocity.h"
#include "common_msgs/MeasurementPosition.h"
#include "common_msgs/MeasurementVelocity.h"
#include "common_msgs/MeasurementPosVel.h"
#include "common_msgs/PixhawkCMD.h"
#include "common_msgs/PixhawkServo.h"
#include "common_msgs/MaxDynamics.h"
#include "common_msgs/Reference.h"
#include "common_msgs/Mission.h"
#include "common_msgs/MissionStatus.h"
#include "common_msgs/PathPlanningStatus.h"
#include "common_msgs/PositionWithCovarianceStamped.h"

// #include "pose_local_parameterization.h"
#include "basic_function.h"
#include "utility.h"
#include "utils.h"





// -----------------------------

class UWB_Utils
{
public:
    struct comp{
        template <typename T>
        inline bool operator()(const T& a, const T& b) const{
            return (a<b);
        }
    };


    template <typename T>
    double robustAverage(std::vector<T> array){
        // the paramter is copied, so that the original data will not be affected.
        const int cut_length = 2;

        if(array.size()<=2*cut_length){
            return 0;
        }

        double sum = 0;
        std::sort(array.begin(), array.end(), comp());
        for(int i=cut_length;i<array.size()-cut_length;++i){
            sum+=array.at(i);
        }

        return sum / (array.size()-2*cut_length);

    }

    template <typename T>
    bool medianAndVariance(std::vector<T> array, double& median, double& variance){
        if(array.size()==0){
            return false;
        }

        // get median
        std::sort(array.begin(), array.end(), comp());
        median = array.at(std::floor(array.size()/2));

        // get mean
        double sum = 0;
        for(int i=0;i<array.size();++i){
            sum+=array.at(i);
        }
        double mean = sum / array.size();


        // get variance
        double variance_sum = 0;
        for(int i=0;i<array.size();++i){
            double res = array.at(i)-mean;
            variance_sum += res*res;
        }
        variance = variance_sum / array.size();

        return true;
    }
};


// -----------------------------


class UWB_Node_ESKF{
public:
    explicit UWB_Node_ESKF(const int node_id);

protected:
    int m_node_id;

    geometry_msgs::PointStamped m_position;
    geometry_msgs::PointStamped m_velocity;
    Eigen::Quaterniond m_orientation;

public:
    inline int getNodeId() const{
        return m_node_id;
    }
    inline geometry_msgs::PointStamped getPosition() const{
        return m_position;
    }
    inline geometry_msgs::PointStamped* getPositionPtr(){
        return &m_position;
    }
    inline geometry_msgs::PointStamped getVelocity() const{
        return m_velocity;
    }

    inline Eigen::Quaterniond getOrientation() const{
        return m_orientation;
    }

    inline void setOrientation(const double x, const double y, const double z, const double w)
    {
        m_orientation.w() = w;
        m_orientation.x() = x;
        m_orientation.y() = y;
        m_orientation.z() = z;
    }

    //设置position
    inline void setPosition(const double x, const double y, const double z){
        m_position.header.stamp = ros::Time::now();

        m_position.point.x = x;
        m_position.point.y = y;
        m_position.point.z = z;
    }

    //设置速度
    inline void setVelocity(const double vx, const double vy, const double vz){
        m_velocity.header.stamp = ros::Time::now();

        m_velocity.point.x = vx;
        m_velocity.point.y = vy;
        m_velocity.point.z = vz;
    }
};

class UWB_Anchor_ESKF : public UWB_Node_ESKF{
public:
    explicit UWB_Anchor_ESKF(const int node_id,
                        const double x, const double y, const double z,
                        const double var_x, const double var_y, const double var_z);
    UWB_Utils uwb_utils;
public:
    std::map<int, std::vector<double>> m_range_map;
    void insertRangeInfo(const common_msgs::UWB_EchoedRangeInfo& msg);
    void insertRangeInfo(const common_msgs::UWB_FullRangeInfo& msg);
    double getAverageRangeToNode(const int anchor_id);
    bool getMedianAndVarianceToNode(const int anchor_id, double& median, double& variance);

    inline void printPosition(){
        std::cout<<m_node_id<<": "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z<<std::endl;
    }
};


class UWB_Mobile_ESKF : public UWB_Node_ESKF{
public:
    explicit UWB_Mobile_ESKF(const int node_id,
                        const std::map<int, boost::shared_ptr<UWB_Anchor_ESKF> > anchor_map,
                        const ros::NodeHandle& nh);

public:
    // stores last correct/accepted measurements
    std::map<int, common_msgs::UWB_FullRangeInfo> m_ranges_to_anchors;
    ros::Time m_last_range_time;

   bool m_is_last_filter_succeed;//上一次滤波是否成功

    // reading statistics, to get effective frequency
    std::map<int, int> m_anchor_reading_counter;
    std::map<int, int> m_anchor_successful_reading_counter;

    // check whether the filter is properly initialized and ready to use
    // 检查滤波器是否被有效初始化
    bool m_is_stablized;
    std::vector<geometry_msgs::PointStamped> m_init_inspector_position_array;


    // ################ from slam pp ################

    double m_innovation_threshold;//threshold
    std::string m_filter_type;
    double m_initialization_innovation_threshold;
    double m_normal_innovation_threshold;
    double m_innovation_threshold_inc_on_succ;
    double m_innovation_threshold_inc_on_fail;

    double m_z_damping_factor;
    double m_Q_scale;
    double m_R_scale;

    // debug
    int m_counter;
    double m_kalman_sigma_a;
    double m_snr_threshold;
    double m_tao_acc_sqrt;
    double m_tao_bias_sqrt;
    // ################ from slam pp ################

    // ### some params ####



    Eigen::Vector3d m_last_position;
    Eigen::Vector3d m_last_velocity;
    Eigen::Matrix3d m_last_rotation;

    Eigen::Vector3d m_pred_position;
    Eigen::Vector3d m_pred_velocity;
    Eigen::Matrix3d m_pred_rotation;
    Eigen::Matrix<double, 15, 15> m_state_covariance;//15 x 15的状态协方差矩阵
    // provided by parameter server
    std::map<int, boost::shared_ptr<UWB_Anchor_ESKF> > m_anchor_map;
    Eigen::Vector3d m_g;

    Eigen::Vector3d m_acceleration_bias;
    Eigen::Vector3d m_angular_bias;
    Eigen::Matrix3d m_rotation;

    std::vector<double> m_td_buf;
    std::vector<Eigen::Vector3d> m_linear_acceleration_buf;
    std::vector<Eigen::Vector3d> m_angular_velocity_buf;

    Eigen::Vector3d acc_0, gyr_0;

    bool m_first_imu;

    // parameters
    ros::NodeHandle m_nh;

public:
    int m_least_anchor_num;

public:

    //setLast
    inline void setLastPosition(const Eigen::Vector3d position)
    {
        m_last_position = position;
        m_pred_position = position;
    }

    inline void setLastVelocity(const Eigen::Vector3d velocity)
    {
        m_last_velocity = velocity;
        m_pred_velocity = velocity;
    }

    inline void setLastRotation(const Eigen::Matrix3d rotation)
    {
        m_last_rotation = rotation;
        m_pred_rotation = rotation;
    }


    inline void setAngularBias(const Eigen::Vector3d angular_bias){
        m_angular_bias = angular_bias;
    }


    inline Eigen::Vector3d getAngularBias() const
    {
        return m_angular_bias;
    }

    inline void setAccelerationBias(const Eigen::Vector3d acceleration_bias){
        m_acceleration_bias = acceleration_bias;

    }

    inline Eigen::Vector3d getAccelerationBias() const
    {
        return m_acceleration_bias;
    }

    inline common_msgs::UWB_FullRangeInfo getRangeInfoToAnchor(const int anchor_id) const{
        return m_ranges_to_anchors.find(anchor_id)->second;
    }

    inline double getRangeMeterToAnchor(const int anchor_id) const{
        return double(m_ranges_to_anchors.find(anchor_id)->second.precisionRangeMm) / 1000.0;
    }

    inline void setRangeInfoToAnchor(const common_msgs::UWB_FullRangeInfo& range_info){
        m_ranges_to_anchors[range_info.responderId] = range_info;
    }

    inline bool getAnchorPosition(const int anchor_id, geometry_msgs::PointStamped& anchor_position) const {
        if(m_anchor_map.find(anchor_id)!=m_anchor_map.end()){
            anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
            return true;
        } else {
            return false;
        }
    }

    inline int getAnchorNumber() const{
        return m_anchor_map.size();
    }

    void fullInitialize(const common_msgs::UWB_FullNeighborDatabase& ndb, const geometry_msgs::PointStamped& position);
    double getAverageRangeErrEst();

    // dynamic innovation threshold, to avoid filter divergence
    void getInnovation();
    // void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    bool filter3DUpdate();
    bool tightlyFGOUpdate();
    bool looselyFGOUpdate();
    void doubleToVector();
    void vectorToDouble();



    // #########################new added by slam_pp ##################################
    // stablization of filter
    bool checkStablization();
    inline bool getStablizationFlag() const{
        return m_is_stablized;
    }
    inline double getInnovationThreshold() const{
        return m_innovation_threshold;
    }

    // statistics for monitoring the ranging status
    //读取Counter的增量
    /**
     * @brief Counter
     * 1. readingCounterInc(const int anchor_id): 这个函数用于增加特定锚节点的读取计数。
     * 如果m_anchor_reading_counter（存储读取计数的映射）中不存在给定的anchor_id，则将其初始化为1，否则将其值增加1。
     * 2. successfulReadingCounterInc(const int anchor_id): 这个函数与readingCounterInc类似，但是它增加的是成功读取的计数，即成功接收到来自特定锚节点的信息。
     * 3. getReadingCount(const int anchor_id): 这个函数用于获取特定锚节点的总读取计数。如果m_anchor_reading_counter中不存在给定的anchor_id，则返回0，否则返回其值。
     * 4. getSuccessfulReadingCount(const int anchor_id): 这个函数与getReadingCount类似，但是它返回的是成功读取的计数。
     */
    inline void readingCounterInc(const int anchor_id){
        //如果无法通过anchor_id找到，则为1；能找到+1
        if(m_anchor_reading_counter.find(anchor_id)==m_anchor_reading_counter.end()){
            m_anchor_reading_counter[anchor_id] = 1;
        } else {
            m_anchor_reading_counter[anchor_id] += 1;
        }
    }
    //成功读取counterInc
    inline void successfulReadingCounterInc(const int anchor_id){
        if(m_anchor_successful_reading_counter.find(anchor_id)==m_anchor_successful_reading_counter.end()){
            m_anchor_successful_reading_counter[anchor_id] = 1;
        } else {
            m_anchor_successful_reading_counter[anchor_id] += 1;
        }
    }
    //得到ReadingCount，数有多少个
    inline int getReadingCount(const int anchor_id){
        if(m_anchor_reading_counter.find(anchor_id)==m_anchor_reading_counter.end()){
            return 0;
        } else {
            return m_anchor_reading_counter.at(anchor_id);
        }
    }

    //成功得到ReadingCout
    inline int getSuccessfulReadingCount(const int anchor_id)
    {
        if(m_anchor_successful_reading_counter.find(anchor_id)==m_anchor_successful_reading_counter.end()){
            return 0;
        } else {
            return m_anchor_successful_reading_counter.at(anchor_id);
        }
    }
    // #########################new added by slam_pp ##################################


    // print
    inline void printPosition(){
        std::cout<<m_node_id<<":"<<std::endl;

        std::cout<<"pose: "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z <<std::endl;//mobile的位置
    }

    inline void printOrientation()
    {
        std::cout << "orientation: \n" << m_orientation.toRotationMatrix() << std::endl;
    }

    inline void printVelocity(){

            std::cout<<"velo: "<<m_velocity.point.x<<", "<<m_velocity.point.y<<", "<<m_velocity.point.z  <<std::endl;

    }

    inline void printAccelerationBias()
    {

            std::cout << "acc bias: " << m_acceleration_bias.x() << ", " << m_acceleration_bias.y() <<", " << m_acceleration_bias.z() << std::endl;
            std::cout << "angular bias: " << m_angular_bias.x() << ", " << m_angular_bias.y() << ", " << m_angular_bias.z() << std::endl;
    }

};





#endif