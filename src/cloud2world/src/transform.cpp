#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

ros::Publisher points_pub;
tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped camera_front_to_world;
geometry_msgs::TransformStamped camera_back_to_world;
// static int accept_count_ = 0;
// static int cloud_count_ = 0;

void synchronizerCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_front,
                          const sensor_msgs::PointCloud2ConstPtr &cloud_back)
{
    // cloud_count_++;
    // if (accept_count_ / cloud_count_ < 0.01) {
    //     accept_count_++;
    // } else {
    //     return;
    // }
    try {
        if (tfBuffer.canTransform("world", "camera_depth_frame", ros::Time(0))) {
            camera_front_to_world = tfBuffer.lookupTransform(
                "world", "camera_depth_frame", ros::Time(0));
        } else {
            return;
        }
        if (tfBuffer.canTransform("world", "camera_depth_frame_back", ros::Time(0))) {
            camera_back_to_world = tfBuffer.lookupTransform(
                "world", "camera_depth_frame_back", ros::Time(0));
        } else {
            return;
        }
    } catch (tf2::TransformException &e) {
        ROS_WARN("Failed to transform camera point cloud: %s", e.what());
    }
    const double APPROX_TIME_THRESHOLD = 0.1;
    double time_diff =
        std::fabs((cloud_front->header.stamp - cloud_back->header.stamp).toSec());
    if (time_diff <= APPROX_TIME_THRESHOLD) {
        // ROS_INFO("Received synchronized messages");
    } else {
        ROS_WARN("Cameras cloud messages are not synchronized");
        return;
    }
    sensor_msgs::PointCloud2 cloud_front_world, cloud_back_world, cloud_merge_world;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_front_world(
        new pcl::PointCloud<pcl::PointXYZ>()),
        pcl_cloud_back_world(new pcl::PointCloud<pcl::PointXYZ>()),
        pcl_cloud_merge_world(new pcl::PointCloud<pcl::PointXYZ>()),
        pcl_cloud_merge_filter_world(new pcl::PointCloud<pcl::PointXYZ>());
    tf2::doTransform(*cloud_front, cloud_front_world, camera_front_to_world);
    tf2::doTransform(*cloud_back, cloud_back_world, camera_back_to_world);
    pcl::fromROSMsg(cloud_front_world, *pcl_cloud_front_world);
    pcl::fromROSMsg(cloud_back_world, *pcl_cloud_back_world);
    *pcl_cloud_merge_world = *pcl_cloud_front_world + *pcl_cloud_back_world;

    //创建voxelGrid对象，下采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;         //创建voxelGrid对象
    voxel_grid.setInputCloud(pcl_cloud_merge_world);  //设置输入点云
    voxel_grid.setLeafSize(
        0.2f, 0.2f, 0.2f);  //设置体素大小，单位是m，这里设置成了 10cm 的立方体
    voxel_grid.filter(*pcl_cloud_merge_filter_world);  //执行滤波，结果保存在cloud_filter中

    // 点云直通滤波
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud_merge_filter_world);  // 设置输入点云
    pass.setFilterFieldName("z");    // 设置过滤的字段为z轴
    pass.setFilterLimits(0.1, 0.5);  // 设置z轴的范围
    pass.filter(*pcl_cloud_merge_filter_world);

    pcl::toROSMsg(*pcl_cloud_merge_filter_world, cloud_merge_world);
    cloud_merge_world.header.frame_id = "world";
    points_pub.publish(cloud_merge_world);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_transform_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_points_sub(
        nh, "/camera/depth/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_back_points_sub(
        nh, "/camera_back/depth/points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(
        MySyncPolicy(10), camera_points_sub, camera_back_points_sub);
    sync.registerCallback(boost::bind(&synchronizerCallback, _1, _2));

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);
    bool can_transform = false;

    ros::spin();

    return 0;
}