#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>

ros::Publisher points_pub;
tf2_ros::Buffer tfBuffer;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    try {
        geometry_msgs::TransformStamped transformStamped =
            tfBuffer.lookupTransform("world", "kinect_v2_cloud_link", ros::Time(0));

        sensor_msgs::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transformStamped);

        // 点云下采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *cloud);
        //创建voxelGrid对象，及相关参数的设置
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;  //创建voxelGrid对象
        voxel_grid.setInputCloud(cloud);           //设置输入点云
        voxel_grid.setLeafSize(
            0.1f, 0.1f, 0.1f);  //设置体素大小，单位是m，这里设置成了 10cm 的立方体
        voxel_grid.filter(*cloud_filtered);  //执行滤波，结果保存在cloud_filter中
        // 点云直通滤波
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);  // 设置输入点云
        pass.setFilterFieldName("z");        // 设置过滤的字段为z轴
        pass.setFilterLimits(0.1, 1.0);  // 设置z轴的范围为 [0.0, 1.0]

        // 执行过滤操作
        pass.filter(*cloud_filtered);
        pcl::toROSMsg(*cloud_filtered, transformed_cloud);
        transformed_cloud.header.frame_id = "world";
        points_pub.publish(transformed_cloud);

    } catch (tf2::TransformException &e) {
        ROS_WARN("Failed to transform point cloud: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_transform_node");
    ros::NodeHandle nh;

    ros::Subscriber points_sub =
        nh.subscribe("/velodyne_points", 10, pointCloudCallback);

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::spin();

    return 0;
}