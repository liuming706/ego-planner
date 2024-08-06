#ifndef _TRAJECTORY_TRACKING_HPP_
#define _TRAJECTORY_TRACKING_HPP_

#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include "plan_manage/plan_container.hpp"
#include "plan_env/grid_map.h"
namespace ego_planner {
class TrajectoryTracker
{
public:
    enum WalkingType { LONGITUDINAL = 0, LATERAL };
    TrajectoryTracker() = default;
    void init(ros::NodeHandle &node, const std::shared_ptr<GridMap> &grid_map);

    void setTrajectory(const LocalTrajData &local_traj);

    void setCurrentState(const nav_msgs::Odometry::ConstPtr &msg);
    bool isTargetStateSafe(const Eigen::Vector3d target_state);

private:
    void robotBoundaryPoint2WorldPoint(const Eigen::Vector3d &current_state,
                                       Eigen::Vector2d &rp, Eigen::Vector2d &wp);
    geometry_msgs::Twist purePursuit(const Eigen::Vector3d &pose,
                                     const Eigen::Vector3d &tar);

    std::pair<double, double> calculate_yaw_for_humanoid_robot(
        double t_cur, Eigen::Vector3d &pos, ros::Time &time_now,
        ros::Time &time_last);

    void cmdCallback(const ros::TimerEvent &e);
    std::unique_ptr<ros::Timer> cmd_timer_{nullptr};
    bool receive_traj_{false};
    double last_yaw_{.0};
    ros::Publisher omni_robot_cmd_pub;
    double time_forward_;
    Eigen::Vector3d current_state_;
    const double dt = 0.1;
    vector<UniformBspline> traj_;
    double traj_duration_;
    ros::Time start_time_;
    int traj_id_;
    std::shared_ptr<GridMap> grid_map_{nullptr};
    WalkingType walking_type_{WalkingType::LONGITUDINAL};
    Eigen::Vector2d w_front_left_p_, w_front_right_p_, w_back_left_p_,
        w_back_right_p_;
    Eigen::Vector2d b_front_left_p_, b_front_right_p_, b_back_left_p_,
        b_back_right_p_;
    double width_ = 0.6, length_ = 1.0;
};

}  // namespace ego_planner

#endif