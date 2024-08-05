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

namespace ego_planner {
class TrajectoryTracker
{
public:
    TrajectoryTracker() = default;
    void init(ros::NodeHandle &node);

    void setTrajectory(const LocalTrajData &local_traj);

    void setCurrentState(const nav_msgs::Odometry::ConstPtr &msg);

private:
    geometry_msgs::Twist purePursuit(const Eigen::Vector3d &pose,
                                     const Eigen::Vector3d &tar);

    std::pair<double, double> calculate_yaw_humanoid_robot(double t_cur,
                                                           Eigen::Vector3d &pos,
                                                           ros::Time &time_now,
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
};

}  // namespace ego_planner

#endif