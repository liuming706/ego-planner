#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "bspline_opt/uniform_bspline.h"
// #include "mpc_tracking/Bspline.h"
#include "ego_planner/Bspline.h"

#include "std_msgs/Empty.h"

#include "mpc_tracking/mpc.h"

// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward{
//     backward::SignalHandling sh;
// }

ros::Publisher cmd_vel_pub, motion_path_pub, predict_path_pub;
nav_msgs::Path predict_path, motion_path;
nav_msgs::Odometry odom;

bool receive_traj = false;
vector<ego_planner::UniformBspline> traj_;
double traj_duration_;
ros::Time start_time;

// double last_yaw;
// double time_forward;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd, traj_real;

ros::Timer control_cmd_pub, path_pub;

const int N = 30;
const double dt = 0.1;

Eigen::Vector3d current_state;

unique_ptr<Mpc> mpc_ptr;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
    // parse pos traj

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (int i = 0; i < msg->knots.size(); ++i) {
        knots(i) = msg->knots[i];
    }
    // for (int i = 0; i < msg->pos_pts.size(); ++i) {
    //     pos_pts(i, 0) = msg->pos_pts[i].x;
    //     pos_pts(i, 1) = msg->pos_pts[i].y;
    //     pos_pts(i, 2) = msg->pos_pts[i].z;
    // }
    // NOTE(lumen): 注意 i 的位置
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    ego_planner::UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    for (int i = 0; i < msg->yaw_pts.size(); ++i) {
        yaw_pts(i, 0) = msg->yaw_pts[i];
    }

    // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    start_time = msg->start_time;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    // traj_.push_back(yaw_traj);
    // traj_.push_back(yaw_traj.getDerivative());

    traj_duration_ = traj_[0].getTimeSum();
    std::cout << "lumen_debug: pos_traj: " << std::endl
              << traj_[0].getControlPoint()
              << ",traj_duration_: " << traj_duration_ << std::endl;
    receive_traj = true;
    std::cout << "lumen_debug: receive_traj = true;" << std::endl;
}

void replanCallback(std_msgs::Empty msg)
{
    /* reset duration */
    const double time_out = 0.01;
    ros::Time time_now = ros::Time::now();
    double t_stop = (time_now - start_time).toSec() + time_out;
    traj_duration_ = min(t_stop, traj_duration_);
}

void odomCallback(const nav_msgs::Odometry &msg)
{
    odom = msg;
    current_state(0) = msg.pose.pose.position.x;
    current_state(1) = msg.pose.pose.position.y;
    current_state(2) = tf2::getYaw(msg.pose.pose.orientation);

    // double yaw1 = tf2::getYaw(msg.pose.pose.orientation);
    // Eigen::Quaterniond quaternion;
    // quaternion.x() = msg.pose.pose.orientation.x;
    // quaternion.y() = msg.pose.pose.orientation.y;
    // quaternion.z() = msg.pose.pose.orientation.z;
    // quaternion.w() = msg.pose.pose.orientation.w;

    // Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
    // double yaw2 = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    // cout << "x:" << current_state(0) << " "
    //      << "y:" << current_state(1) << endl;
    // cout << "yaw1:" << current_state(2) << endl;
    // cout << "yaw2:" << yaw2 << endl;
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

void publish_control_cmd(const ros::TimerEvent &e)
{
    if (!receive_traj) return;

    ros::Time time_now = ros::Time::now();

    double t_cur = (time_now - start_time).toSec();

    Eigen::Vector3d pos, vel, acc, pos_f;
    double yaw, yawdot;

    Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N, 3);
    static ros::Time time_last = ros::Time::now();
    if (t_cur + (N - 1) * dt <= traj_duration_ && t_cur > 0) {
        std::cout << "lumen_debug: 1;" << std::endl;
        for (int i = 0; i < N; ++i) {
            pos = traj_[0].evaluateDeBoorT(t_cur + i * dt);
            vel = traj_[1].evaluateDeBoorT(t_cur + i * dt);
            acc = traj_[2].evaluateDeBoorT(t_cur + i * dt);
            yaw = calculate_yaw(t_cur, pos, time_now, time_last).first;
            // yawdot = traj[4].evaluateDeBoorT(t_cur + i * dt)[0];

            desired_state(i, 0) = pos[0];
            desired_state(i, 1) = pos[1];
            desired_state(i, 2) = yaw;
        }

    } else if (t_cur + (N - 1) * dt > traj_duration_ && t_cur < traj_duration_) {
        std::cout << "lumen_debug: 2;" << std::endl;
        int more_num = (t_cur + (N - 1) * dt - traj_duration_) / dt;
        for (int i = 0; i < N - more_num; ++i) {
            pos = traj_[0].evaluateDeBoorT(t_cur + i * dt);
            vel = traj_[1].evaluateDeBoorT(t_cur + i * dt);
            acc = traj_[2].evaluateDeBoorT(t_cur + i * dt);
            yaw = calculate_yaw(t_cur + i * dt, pos, time_now, time_last).first;
            // yaw = traj[3].evaluateDeBoorT(t_cur + i * dt)[0];
            // yawdot = traj[4].evaluateDeBoorT(t_cur + i * dt)[0];

            desired_state(i, 0) = pos(0);
            desired_state(i, 1) = pos(1);
            desired_state(i, 2) = yaw;
        }
        for (int i = N - more_num; i < N; ++i) {
            pos = traj_[0].evaluateDeBoorT(traj_duration_);
            vel.setZero();
            acc.setZero();
            yaw = calculate_yaw(traj_duration_, pos, time_now, time_last).first;
            // yaw = traj[3].evaluateDeBoorT(traj_duration)[0];
            // yawdot = traj[4].evaluateDeBoorT(traj_duration)[0];

            desired_state(i, 0) = pos(0);
            desired_state(i, 1) = pos(1);
            desired_state(i, 2) = yaw;
        }
    } else if (t_cur >= traj_duration_) {
        std::cout << "t_cur ,traj_duration:" << t_cur << " ," << traj_duration_
                  << std::endl;
        std::cout << "lumen_debug: 3;" << std::endl;
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();
        yaw = calculate_yaw(traj_duration_, pos, time_now, time_last).first;
        // yaw = traj[3].evaluateDeBoorT(traj_duration)[0];
        // yawdot = traj[4].evaluateDeBoorT(traj_duration)[0];
        for (int i = 0; i < N; ++i) {
            desired_state(i, 0) = pos(0);
            desired_state(i, 1) = pos(1);
            desired_state(i, 2) = yaw;
        }
    } else {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_now;

    auto result = mpc_ptr->solve(current_state, desired_state);

    geometry_msgs::Twist cmd;
    cmd.linear.x = result[0];
    cmd.linear.y = result[1];
    // cmd.angular.z = result[1];
    cmd_vel_pub.publish(cmd);
    // cout << "u:" << result[0] << " " << "r:" << result[1] << endl;

    predict_path.header.frame_id = "world";
    predict_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::Point pt;
    for (int i = 2; i < result.size(); i += 2) {
        pose_msg.pose.position.x = result[i];
        pose_msg.pose.position.y = result[i + 1];
        predict_path.poses.push_back(pose_msg);
    }
    predict_path_pub.publish(predict_path);

    predict_path.poses.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_tracking_node");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/motion_path", 1);

    ros::Subscriber odom_sub = nh.subscribe("/world", 1, &odomCallback);
    ros::Subscriber bspline_sub =
        nh.subscribe("planning/bspline", 10, bsplineCallback);
    // ros::Subscriber replan_sub = nh.subscribe("planning/replan", 10, replanCallback);

    control_cmd_pub = nh.createTimer(ros::Duration(0.1), publish_control_cmd);

    nh.param("traj_server/time_forward", time_forward_, -1.0);
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;
    ros::spin();
    return 0;
}
