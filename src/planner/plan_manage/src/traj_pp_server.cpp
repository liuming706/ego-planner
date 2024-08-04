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
ros::Publisher pos_cmd_pub;
ros::Publisher omni_robot_cmd_pub;
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

const double dt = 0.1;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
Eigen::Vector3d current_state_;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_state_(0) = msg->pose.pose.position.x;
    current_state_(1) = msg->pose.pose.position.y;
    current_state_(2) = tf2::getYaw(msg->pose.pose.orientation);
}
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
    // parse pos traj

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
        knots(i) = msg->knots[i];
    }
    // NOTE(lm)： 里程计坐标系下描述的局部路径点
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }
    // // 本体坐标系下描述的局部坐标点
    // for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    //     Eigen::Matrix<double, 4, 1> msg_pos;
    //     msg_pos << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z, 1;
    //     Eigen::Vector4d pose_in_body = body2world.inverse() * msg_pos;
    //     pos_pts(0, i) = pose_in_body(0);
    //     pos_pts(1, i) = pose_in_body(1);
    //     pos_pts(2, i) = pose_in_body(2);
    // }

    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    //   yaw_pts(i, 0) = msg->yaw_pts[i];
    // }

    // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());

    traj_duration_ = traj_[0].getTimeSum();
    std::cout << "lumen_debug: traj_server traj_duration_:" << traj_duration_
              << std::endl;
    std::cout << "lumen_debug: traj_server traj_[0].getControlPoint()" << std::endl
              << traj_[0].getControlPoint() << std::endl;
    receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos,
                                        ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                              ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                              : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();

    if (yaw_temp - last_yaw_ > PI) {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change) {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI) yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        } else {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    } else if (yaw_temp - last_yaw_ < -PI) {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change) {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI) yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        } else {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    } else {
        if (yaw_temp - last_yaw_ < -max_yaw_change) {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI) yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        } else if (yaw_temp - last_yaw_ > max_yaw_change) {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI) yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        } else {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

std::pair<double, double> calculate_yaw_humanoid_robot(double t_cur,
                                                       Eigen::Vector3d &pos,
                                                       ros::Time &time_now,
                                                       ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                              ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                              : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    yaw = yaw_temp;
    // double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now -
    // time_last).toSec(); if (yaw_temp - last_yaw_ > PI) {
    //     if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change) {
    //         yaw = last_yaw_ - max_yaw_change;
    //         if (yaw < -PI) yaw += 2 * PI;

    //         yawdot = -YAW_DOT_MAX_PER_SEC;
    //     } else {
    //         yaw = yaw_temp;
    //         if (yaw - last_yaw_ > PI)
    //             yawdot = -YAW_DOT_MAX_PER_SEC;
    //         else
    //             yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    //     }
    // } else if (yaw_temp - last_yaw_ < -PI) {
    //     if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change) {
    //         yaw = last_yaw_ + max_yaw_change;
    //         if (yaw > PI) yaw -= 2 * PI;

    //         yawdot = YAW_DOT_MAX_PER_SEC;
    //     } else {
    //         yaw = yaw_temp;
    //         if (yaw - last_yaw_ < -PI)
    //             yawdot = YAW_DOT_MAX_PER_SEC;
    //         else
    //             yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    //     }
    // } else {
    //     if (yaw_temp - last_yaw_ < -max_yaw_change) {
    //         yaw = last_yaw_ - max_yaw_change;
    //         if (yaw < -PI) yaw += 2 * PI;

    //         yawdot = -YAW_DOT_MAX_PER_SEC;
    //     } else if (yaw_temp - last_yaw_ > max_yaw_change) {
    //         yaw = last_yaw_ + max_yaw_change;
    //         if (yaw > PI) yaw -= 2 * PI;

    //         yawdot = YAW_DOT_MAX_PER_SEC;
    //     } else {
    //         yaw = yaw_temp;
    //         if (yaw - last_yaw_ > PI)
    //             yawdot = -YAW_DOT_MAX_PER_SEC;
    //         else if (yaw - last_yaw_ < -PI)
    //             yawdot = YAW_DOT_MAX_PER_SEC;
    //         else
    //             yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    //     }
    // }

    // if (fabs(yaw - last_yaw_) <= max_yaw_change)
    //     yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // nieve LPF
    // yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

float quaternion2Yaw(geometry_msgs::Quaternion orientation)
{
    double q0 = orientation.x;
    double q1 = orientation.y;
    double q2 = orientation.z;
    double q3 = orientation.w;

    float yaw = atan2(2.0 * (q2 * q3 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    return yaw;
}

geometry_msgs::Twist purePursuit(const Eigen::Vector3d &pose,
                                 const Eigen::Vector3d &tar)
{
    // PID Control
    double delta_x = tar[0] - pose[0];
    double delta_y = tar[1] - pose[1];
    double delta_th = tar[2] - pose[2];
    std::cout << "lumen_DEBUG: " << tar[2] << " - " << pose[2] << " = "
              << delta_th << std::endl;
    if (delta_th > M_PI) {
        delta_th -= 2 * M_PI;
    } else if (delta_th < -M_PI) {
        delta_th += 2 * M_PI;
    }

    double Kx = 5;
    double Ky = 5;
    double Kw = 2;

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

void cmdCallback(const ros::TimerEvent &e)
{
    /* no publishing before receive traj_ */
    if (!receive_traj_) return;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    Eigen::Vector3d desire_pos(Eigen::Vector3d::Zero());
    Eigen::Vector3d desire_vel(Eigen::Vector3d::Zero());
    static ros::Time time_last = ros::Time::now();

    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        desire_pos = traj_[0].evaluateDeBoorT(t_cur);
        desire_vel = traj_[1].evaluateDeBoorT(t_cur);
        desire_pos[2] =
            calculate_yaw_humanoid_robot(t_cur, desire_pos, time_now, time_last).first;
        desire_vel[2] =
            calculate_yaw_humanoid_robot(t_cur, desire_pos, time_now, time_last).second;
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
        std::cout << "desire_pos: " << desire_pos << std::endl;
        std::cout << "desire_vel: " << desire_vel << std::endl;
        // 未加轨迹跟踪器
        // geometry_msgs::Twist cmd;
        // cmd.linear.x = desire_vel[0] * cos(current_state_(2)) +
        //                desire_vel[1] * sin(current_state_(2));
        // cmd.linear.y = -desire_vel[0] * sin(current_state_(2)) +
        //                desire_vel[1] * cos(current_state_(2));
        // cmd.angular.z = desire_vel[2];
        // omni_robot_cmd_pub.publish(cmd);
        // 纯追踪轨迹跟踪器
        omni_robot_cmd_pub.publish(purePursuit(current_state_, desire_pos));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");

    ros::Subscriber bspline_sub =
        node.subscribe("planning/bspline", 10, bsplineCallback);
    ros::Subscriber odom_sub = node.subscribe("odom_world", 10, odomCallback);

    pos_cmd_pub =
        node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    if (ROBOT_TYPE == "ground_robot") {
        omni_robot_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    }

    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

    /* control parameter */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

    nh.param("traj_server/time_forward", time_forward_, -1.0);
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    ros::spin();

    return 0;
}