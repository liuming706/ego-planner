#include <plan_manage/trajectory_tracker.hpp>

namespace ego_planner {

void TrajectoryTracker::init(ros::NodeHandle &node,
                             const std::shared_ptr<GridMap> &grid_map)
{
    grid_map_ = grid_map;
    node.param("traj_server/time_forward", time_forward_, -1.0);

    double width_ = 0.6, length_ = 1.0;
    b_front_left_p_ = {width_ / 2, length_ / 2};
    b_front_right_p_ = {width_ / 2, -length_ / 2};
    b_back_left_p_ = {-width_ / 2, length_ / 2};
    b_back_right_p_ = {-width_ / 2, -length_ / 2};

    if (ROBOT_TYPE == "ground_robot") {
        omni_robot_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    }
    cmd_timer_ = std::make_unique<ros::Timer>(node.createTimer(
        ros::Duration(0.1), &TrajectoryTracker::cmdCallback, this));
}

void TrajectoryTracker::setTrajectory(const LocalTrajData &local_traj)

{
    start_time_ = local_traj.start_time_;
    traj_.clear();
    traj_.push_back(local_traj.position_traj_);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    receive_traj_ = true;
}
void TrajectoryTracker::setCurrentState(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_state_(0) = msg->pose.pose.position.x;
    current_state_(1) = msg->pose.pose.position.y;
    current_state_(2) = tf2::getYaw(msg->pose.pose.orientation);
}

void TrajectoryTracker::robotBoundaryPoint2WorldPoint(
    const Eigen::Vector3d &current_state, Eigen::Vector2d &bp, Eigen::Vector2d &wp)
{
    wp[0] = current_state(0) + bp(0) * cos(current_state(2)) -
            bp(1) * sin(current_state(2));
    wp[1] = current_state(1) + bp(0) * sin(current_state(2)) +
            bp(1) * cos(current_state(2));
}

bool TrajectoryTracker::isTargetStateSafe(const Eigen::Vector3d target_state)
{
    robotBoundaryPoint2WorldPoint(target_state, b_front_left_p_, w_front_left_p_);
    robotBoundaryPoint2WorldPoint(target_state, b_front_right_p_, w_front_right_p_);
    robotBoundaryPoint2WorldPoint(target_state, b_back_left_p_, w_back_left_p_);
    robotBoundaryPoint2WorldPoint(target_state, b_back_right_p_, w_back_right_p_);
    if (grid_map_->getInflateOccupancy2d(w_front_left_p_) ||
        grid_map_->getInflateOccupancy2d(w_front_right_p_) ||
        grid_map_->getInflateOccupancy2d(w_back_left_p_) ||
        grid_map_->getInflateOccupancy2d(w_back_right_p_)) {
        return false;
    } else {
        return true;
    }
}

std::pair<double, double> TrajectoryTracker::calculate_yaw_for_humanoid_robot(
    double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double t_forward = t_cur + time_forward_;

    Eigen::Vector3d ahead_pose = t_forward <= traj_duration_
                                     ? traj_[0].evaluateDeBoorT(t_forward)
                                     : traj_[0].evaluateDeBoorT(traj_duration_);
    t_forward += time_forward_;

    Eigen::Vector3d dir = ahead_pose - pos;
    yaw = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    ahead_pose(2) = yaw;

    if (walking_type_ == WalkingType::LONGITUDINAL) {
        if (isTargetStateSafe(ahead_pose)) {
            std::cout << "lumen_debug:  WalkingType::LONGITUDINAL" << std::endl;
        } else {
            yaw += M_PI / 2;
            walking_type_ = WalkingType::LATERAL;
            std::cout << "lumen_debug: WalkingType::LATERAL" << std::endl;
        }
    } else if (walking_type_ == WalkingType::LATERAL) {
        if (isTargetStateSafe(ahead_pose)) {
            ahead_pose = current_state_;
            ahead_pose(2) = yaw;
            if (isTargetStateSafe(ahead_pose)) {
                walking_type_ = WalkingType::LATERAL;
                std::cout << "lumen_debug:  WalkingType::LONGITUDINAL" << std::endl;

            } else {
                yaw += M_PI / 2;
                std::cout << "lumen_debug: WalkingType::LATERAL" << std::endl;
            }
        } else {
            yaw += M_PI / 2;
            std::cout << "lumen_debug: WalkingType::LATERAL" << std::endl;
        }
    }

    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2 * M_PI;
    }
    last_yaw_ = yaw;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = 0;

    return yaw_yawdot;
}

geometry_msgs::Twist TrajectoryTracker::purePursuit(const Eigen::Vector3d &pose,
                                                    const Eigen::Vector3d &tar)
{
    // PID Control
    double delta_x = tar[0] - pose[0];
    double delta_y = tar[1] - pose[1];
    double delta_th = tar[2] - pose[2];
    if (delta_th > M_PI) {
        delta_th -= 2 * M_PI;
    } else if (delta_th < -M_PI) {
        delta_th += 2 * M_PI;
    }

    double Kx = 5;
    double Ky = 5;
    double Kw = 2 * 2;

    geometry_msgs::Twist control;
    if (sqrt(delta_x * delta_x + delta_y * delta_y) > 0.05) {
        control.linear.x =
            Kx * delta_x * cos(pose[2]) + Ky * delta_y * sin(pose[2]);
        control.linear.y =
            -Kx * delta_x * sin(pose[2]) + Ky * delta_y * cos(pose[2]);
        // // 角速度限幅，防止振荡
        // control.angular.z = min(Kw * delta_th, control.angular.z + 0.2);
        // control.angular.z = max(control.angular.z, control.angular.z - 0.2);
        control.angular.z = Kw * delta_th;

    } else {
        cout << "Done!!" << endl;
        control.linear.x = 0.0;
        control.linear.y = 0.0;
        control.angular.z = 0.0;
    }
    return control;
}

void TrajectoryTracker::cmdCallback(const ros::TimerEvent &e)
{
    if (!receive_traj_) return;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    Eigen::Vector3d desire_pos(Eigen::Vector3d::Zero());
    Eigen::Vector3d desire_vel(Eigen::Vector3d::Zero());
    static ros::Time time_last = ros::Time::now();

    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        desire_pos = traj_[0].evaluateDeBoorT(t_cur);
        desire_vel = traj_[1].evaluateDeBoorT(t_cur);
        desire_pos[2] = calculate_yaw_for_humanoid_robot(t_cur, desire_pos,
                                                         time_now, time_last)
                            .first;
        desire_vel[2] = calculate_yaw_for_humanoid_robot(t_cur, desire_pos,
                                                         time_now, time_last)
                            .second;
    } else if (t_cur >= traj_duration_) {
        /* hover when finish traj_ */
        desire_pos = traj_[0].evaluateDeBoorT(traj_duration_);
        desire_vel.setZero();
        desire_pos[2] = last_yaw_;
    } else {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_now;

    if (omni_robot_cmd_pub.getNumSubscribers() != 0) {
        omni_robot_cmd_pub.publish(purePursuit(current_state_, desire_pos));
    }
}

}  // namespace ego_planner

// namespace ego_planner