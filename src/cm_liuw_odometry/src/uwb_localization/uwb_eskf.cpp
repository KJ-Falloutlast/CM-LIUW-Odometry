#include "uwb_process/uwb_eskf.h"

UWB_Node_ESKF::UWB_Node_ESKF(const int node_id):m_node_id(node_id){}

// ---------------------------------------------
// anchor相关
UWB_Anchor_ESKF::UWB_Anchor_ESKF(const int node_id,
                              const double x, const double y, const double z,
                              const double var_x, const double var_y, const double var_z)
    :UWB_Node_ESKF(node_id)
{
    setPosition(x,y,z);

}

void UWB_Anchor_ESKF::insertRangeInfo(const common_msgs::UWB_FullRangeInfo& msg){
    // only insert range info that is requested by myself.

    int requester_id = msg.nodeId;
    int responder_id = msg.responderId;
    double range = double(msg.precisionRangeMm) / 1000.0;

    if(m_node_id==requester_id){
        // if haven't seen the responder yet, create it
        if(m_range_map.find(responder_id)==m_range_map.end()){
            m_range_map.insert(std::pair<int, std::vector<double> >(responder_id, std::vector<double>()));
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        } else {
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        }
    }
}

double UWB_Anchor_ESKF::getAverageRangeToNode(const int anchor_id){
    // check whether the anchor_id has been seen
    std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
    if(it==m_range_map.end()){
        return 0;
    } else {
        // check the size of the vector
        if(it->second.size()==0){
            return 0;
        } else {
            return uwb_utils.robustAverage<double>(it->second);
        }
    }
}

bool UWB_Anchor_ESKF::getMedianAndVarianceToNode(const int anchor_id, double &median, double &variance){
    std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
    if(it==m_range_map.end()){
        return false;
    } else {
        if(it->second.size()==0){
            return false;
        } else {
            uwb_utils.medianAndVariance<double>(it->second, median, variance);
            return true;
        }
    }
}

// Mobile
UWB_Mobile_ESKF::UWB_Mobile_ESKF(const int node_id,
                              const std::map<int, boost::shared_ptr<UWB_Anchor_ESKF> > anchor_map,
                              const ros::NodeHandle& nh)
    :UWB_Node_ESKF(node_id),
      m_anchor_map(anchor_map),
      m_nh(nh)
{
    m_nh.param<std::string>("/uwb/filter_type", m_filter_type, std::string("EKF"));//EKF为滤波方式
    m_nh.param<double>("/uwb/z_damping_factor",m_z_damping_factor, 0.1);//Q
    m_nh.param<double>("/uwb/Q_scale",m_Q_scale, 1);//Q
    m_nh.param<double>("/uwb/R_scale", m_R_scale, 1);//R
    m_nh.param<double>("/uwb/initialization_innovation_threshold", m_initialization_innovation_threshold, 50);//初始化增益因子阈值
    m_nh.param<double>("/uwb/normal_innovation_threshold", m_normal_innovation_threshold, 2);//最大的innovation_threshold(判断是否收敛的标准)
    m_nh.param<double>("/uwb/innovation_threshold_inc_on_succ", m_innovation_threshold_inc_on_succ, 0);//成功增益阈值
    m_nh.param<double>("/uwb/innovation_threshold_inc_on_fail", m_innovation_threshold_inc_on_fail, 0);//失败增益阈值
    m_nh.param<double>("/uwb/tao_acc_sqrt", m_tao_acc_sqrt, 0.3);//acc_sqrt
    m_nh.param<double>("/uwb/tao_bias_sqrt", m_tao_bias_sqrt, 0.001);//ba_sqrt

}

// 完全初始化
void UWB_Mobile_ESKF::fullInitialize(const common_msgs::UWB_FullNeighborDatabase& ndb, const geometry_msgs::PointStamped& position){
    setPosition(position.point.x, position.point.y, position.point.z);//得到m_position
    // setVelocity(0,0,0);
    // setAccelerationBias(Eigen::Vector3d(0,0,0));
    // setAngularBias(Eigen::Vector3d(0,0,0));

    m_ranges_to_anchors.clear();

    common_msgs::UWB_FullRangeInfo zero_range_info;
    for(int i=0;i<ndb.numNeighborEntries;++i){
        int anchor_id = ndb.neighbors.at(i).nodeId;

        zero_range_info.header.stamp = ndb.header.stamp;
        zero_range_info.responderId = anchor_id;
        zero_range_info.precisionRangeMm = ndb.neighbors.at(i).rangeMm;
        zero_range_info.precisionRangeErrEst = ndb.neighbors.at(i).rangeErrorEstimate;

        // avoid invalid ndb entry
        if(zero_range_info.precisionRangeMm>50)//rangeError必须大于50mm, 仿真环境为100
        {
            // ROS_INFO("anchor_id:%d inserted", anchor_id);//是否插入成功
            m_ranges_to_anchors.insert(std::make_pair(anchor_id,zero_range_info));
        }
    }

    m_last_range_time = ros::Time::now();
}

double UWB_Mobile_ESKF::getAverageRangeErrEst()
{
    double sum_range_err = 0.0;
    std::map<int, common_msgs::UWB_FullRangeInfo>::iterator iter;
    for(iter=m_ranges_to_anchors.begin();iter!=m_ranges_to_anchors.end();++iter)
    {
        sum_range_err += iter->second.precisionRangeErrEst;
    }

    return sum_range_err/m_ranges_to_anchors.size();
}


/**
 * @brief 调整创新阈值
 * @note getInnovation 函数用于根据系统稳定性动态调整创新性阈值。首先，它检查系统是否稳定，
 * 如果不稳定，则将初始创新性阈值设置为初始化增益因子阈值，并调用 checkStablization 函数来检查系统的稳定性。
 * 如果系统稳定，根据先前的稳定性标志和上次滤波是否成功的情况，动态调整创新性阈值。最后，确保创新性阈值在预设的上下限范围内。
 *
 */
void UWB_Mobile_ESKF::getInnovation()
{
    bool previous_is_stablized_flag = m_is_stablized;

    //如果未稳定，则检查稳定性
    if(false==m_is_stablized){
        m_innovation_threshold = m_initialization_innovation_threshold;
        m_is_stablized = checkStablization();
    }

    //如果稳定，控制m_innovation_threshold大小
    if(true==m_is_stablized){
        if(previous_is_stablized_flag==false)
        {
            m_innovation_threshold = m_normal_innovation_threshold;
        }
        // dynamic innovation
        if(true==m_is_last_filter_succeed){
            m_innovation_threshold += m_innovation_threshold_inc_on_succ;
        } else {
            m_innovation_threshold += m_innovation_threshold_inc_on_fail;
        }

        // upper and lower limit
        if(m_innovation_threshold < m_normal_innovation_threshold)
        {
            m_innovation_threshold = m_normal_innovation_threshold;
        } else if(m_innovation_threshold > m_initialization_innovation_threshold)
        {
            m_innovation_threshold = m_initialization_innovation_threshold;
        }
    }

}

/**
 * @brief 稳定性检查
 * @note: 这个函数的主要用途是在机器人启动时，检查机器人的位置变化是否足够小，以判断系统是否已经稳定。
 * 函数的主要步骤如下：
    1. 如果系统已经稳定（m_is_stablized为true），则直接返回true。
    2. 将当前机器人的位置信息（this->getPosition()）添加到m_init_inspector_position_array数组中。
    3. 从m_init_inspector_position_array数组中获取最近5秒内的有效位置信息，存储在valid_position_array数组中。
    有效位置信息的判断是通过比较当前时间戳和位置信息的时间戳，如果时间差小于5秒，则认为是有效的。
    4. 如果valid_position_array数组的大小小于10，说明还没有足够的位置信息来判断系统是否稳定，直接返回false。
    5. 如果valid_position_array数组的大小大于等于10，计算这些位置信息的均值（x_mean，y_mean，z_mean）。
    6. 计算每个位置信息与均值的差的平方和，然后计算这个平方和的均值的平方根，得到sigma_x，sigma_y，sigma_z。这些值可以用来表示位置信息的变化程度。
    7. 如果sigma_x，sigma_y，sigma_z都小于0.5，说明机器人的位置变化足够小，可以认为系统已经稳定，返回true。否则，返回false。
 */
//关键
bool UWB_Mobile_ESKF::checkStablization(){
    if(m_is_stablized==true){
        return true;
    }
    //将position加入到array中
    m_init_inspector_position_array.push_back(this->getPosition());

    // 得到有效的position_array
    std::vector<geometry_msgs::PointStamped> valid_position_array;

    // 初始化
    for(int i=m_init_inspector_position_array.size()-1;i>=0;--i)
    {
        //得到有效的position
        geometry_msgs::PointStamped position = m_init_inspector_position_array.at(i);
        if(ros::Time::now().toSec() - position.header.stamp.toSec() < 5){
            valid_position_array.push_back(position);//如果当前时间戳和position时间戳 < 5，则加入进去
        } else {
            break;
        }
    }


    if(valid_position_array.size()<10){
        return false;
    } else {
        // get x,y,z mean
        float x_sum=0;
        float y_sum=0;
        float z_sum=0;
        for(int i=0;i<valid_position_array.size();++i){
            geometry_msgs::PointStamped position = valid_position_array.at(i);
            x_sum += position.point.x;
            y_sum += position.point.y;
            z_sum += position.point.z;
        }
        //得到x,y,z均值
        float x_mean = x_sum / float(valid_position_array.size());
        float y_mean = y_sum / float(valid_position_array.size());
        float z_mean = z_sum / float(valid_position_array.size());

        float squared_residual_error_sum_x = 0;
        float squared_residual_error_sum_y = 0;
        float squared_residual_error_sum_z = 0;

        //得到均方根值
        for(int i=0;i<valid_position_array.size();++i)
        {
            geometry_msgs::PointStamped position = valid_position_array.at(i);
            squared_residual_error_sum_x += std::pow(position.point.x-x_mean, 2);
            squared_residual_error_sum_y += std::pow(position.point.y-y_mean, 2);
            squared_residual_error_sum_z += std::pow(position.point.z-z_mean, 2);
        }
        //得到sigma
        float sigma_x = std::sqrt( squared_residual_error_sum_x / float(valid_position_array.size()) );
        float sigma_y = std::sqrt( squared_residual_error_sum_y / float(valid_position_array.size()) );
        float sigma_z = std::sqrt( squared_residual_error_sum_z / float(valid_position_array.size()) );


        ROS_INFO_THROTTLE(1.0, "[Check Init] Sigma in the last 5 second (x, y, z): %f, %f, %f", sigma_x, sigma_y, sigma_z);
        if(sigma_x<0.5 && sigma_y<0.5 && sigma_z<0.5){
            return true;
        } else {
            return false;
        }
    }
}