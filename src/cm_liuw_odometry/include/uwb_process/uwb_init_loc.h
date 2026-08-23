#ifndef _UWB_INIT_LOC_H_
#define _UWB_INIT_LOC_H_

#include "anchor_coordinate_factor.h"
#include "anchor_distance_factor.h"
#include "anchor_triangulation_factor.h"
#include "uwb_eskf.h"

class UWB_Loc_Init{
public:
    // this class should be constructed and destructed before creating the range info callback
    explicit UWB_Loc_Init(const common_msgs::UWB_FullNeighborDatabase& ndb,
                          boost::shared_ptr<UWB_Mobile_ESKF> p_mobile):
        m_ndb(ndb),m_p_mobile(p_mobile){
        m_mobile_position = m_p_mobile->getPosition();
    }
    ~UWB_Loc_Init(){
        // ROS_INFO("UWB Localization Initialization module is now destructed.");
    }

private:
    boost::shared_ptr<UWB_Mobile_ESKF> m_p_mobile;
    geometry_msgs::PointStamped m_mobile_position;
    common_msgs::UWB_FullNeighborDatabase m_ndb;
    ceres::Problem m_problem;

public:
    bool initializeMobileByCeres();
    void buildOptimizationProblem(const common_msgs::UWB_FullNeighborDatabase& ndb);
    bool solveOptimizationProblem();
};

#endif