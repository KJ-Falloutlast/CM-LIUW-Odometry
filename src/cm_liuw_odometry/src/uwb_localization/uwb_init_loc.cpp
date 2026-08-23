#include "uwb_process/uwb_init_loc.h"

bool UWB_Loc_Init::initializeMobileByCeres(){
    buildOptimizationProblem(m_ndb);
    if( solveOptimizationProblem() ){
        m_p_mobile->fullInitialize(m_ndb, m_mobile_position);
        // ROS_INFO("Innitialization with Ceres OK. %f, %f, %f",
        //          m_mobile_position.point.x,
        //          m_mobile_position.point.y,
        //          m_mobile_position.point.z);
        //Innitialization with Ceres OK. -0.234598, -0.161658, 0.479096
        return true;
    } else {
        ROS_INFO("Fail to initialize with Ceres.");
        return false;
    }
}

void UWB_Loc_Init::buildOptimizationProblem(const common_msgs::UWB_FullNeighborDatabase& ndb){
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
    // ROS_INFO("buildOptimizationProblem");
    //利用三角测距方法初始化
    for(int i=0;i<m_ndb.numNeighborEntries;++i){
        geometry_msgs::PointStamped anchor_position;
        if(m_p_mobile->getAnchorPosition(m_ndb.neighbors.at(i).nodeId, anchor_position)){
            ceres::CostFunction* cost_function = AnchorTriangulationErrorTerm::Create(anchor_position.point.x,
                                                                                             anchor_position.point.y,
                                                                                             anchor_position.point.z,
                                                                                             m_ndb.neighbors.at(i).rangeMm/1000.0,
                                                                                             m_ndb.neighbors.at(i).rangeErrorEstimate/1000.0*5);
            m_problem.AddResidualBlock(cost_function, loss_function,
                                       &(m_mobile_position.point.x),
                                       &(m_mobile_position.point.y),
                                       &(m_mobile_position.point.z));
        }
    }
}

bool UWB_Loc_Init::solveOptimizationProblem(){
    // ROS_INFO("solveOptimizationProblem");
    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &m_problem, &summary);

    // std::cout<<"\n"<<summary.BriefReport()<<"\n";
//    std::cout << "Initial cost: " << summary.initial_cost << "  Final cost: "<< summary.final_cost << '\n';

    return summary.IsSolutionUsable();
    //return true;
}