#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/TransformStamped.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// #include <pcl/filters/statistical_outlier_removal.h>

#include <std_srvs/Trigger.h>

class PointCloudRelay
{
public:
    PointCloudRelay(ros::NodeHandle &nh) : nh_(nh)
    {
        // Parameters
        nh_.param<std::string>("input_topic", input_topic_, "/input_pointcloud");
        nh_.param<std::string>("output_topic", output_topic_, "/output_pointcloud");
        nh_.param<std::string>("input_frame", input_frame_, "input_frame");
        nh_.param<std::string>("output_frame", output_frame_, "output_frame");

        // Setup publisher and service clent
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        srv_ = nh_.advertiseService("/pointcloud_relay/capture_pointcloud", &PointCloudRelay::processCallback, this);

        tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);

        ROS_INFO("PointCloudRelay initialized. Listening on [%s], publishing on [%s]", input_topic_.c_str(), output_topic_.c_str());
    }

private:
    const double waiting_time = 5.0;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::ServiceServer srv_;
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    std::string input_topic_, output_topic_, input_frame_, output_frame_;

    sensor_msgs::PointCloud2 pointCloudFilter(const sensor_msgs::PointCloud2ConstPtr &msg)
    {

        geometry_msgs::TransformStamped tf_msg;
        try
        {
            tf_msg = tf_buffer.lookupTransform(output_frame_, input_frame_, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_STREAM("TF failed: " << ex.what());
            sensor_msgs::PointCloud2 final_msg = *msg;
            final_msg.header.stamp = ros::Time::now();
            final_msg.header.frame_id = input_frame_;
            return final_msg;
        }

        // Convert msg to PCL
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud_in);

        // Apply rotation only
        Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf_msg);
        Eigen::Affine3f rot_only = Eigen::Affine3f::Identity();
        rot_only.linear() = tf_eigen.linear().cast<float>(); // rotation only

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_in, *cloud_rotated, rot_only);

        // Crop Box
        pcl::CropBox<pcl::PointXYZRGB> crop;
        crop.setInputCloud(cloud_rotated);
        crop.setMin(Eigen::Vector4f(0.0, -0.5, -2.0, 1.0));
        crop.setMax(Eigen::Vector4f(2.0, 0.5, 0.0, 1.0));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGB>);
        crop.filter(*cropped);

        float leafsize = 0.05;
        // Downsample
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(cropped);
        voxel.setLeafSize(leafsize, leafsize, leafsize);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        voxel.filter(*downsampled);

        // Color by height
        float z_min = std::numeric_limits<float>::max(), z_max = -std::numeric_limits<float>::max();
        for (const auto &pt : downsampled->points)
        {
            if (!std::isnan(pt.z))
            {
                z_min = std::min(z_min, pt.z);
                z_max = std::max(z_max, pt.z);
            }
        }
        for (auto &pt : downsampled->points)
        {
            float ratio = (pt.z - z_min) / (z_max - z_min + 1e-6f);
            pt.r = static_cast<uint8_t>(255 * ratio);
            pt.g = 0;
            pt.b = static_cast<uint8_t>(255 * (1 - ratio));
        }

        // Apply translation after filtering
        Eigen::Translation3f translation(
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z);
        Eigen::Affine3f trans_only(translation);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*downsampled, *cloud_final, trans_only);

        // Convert back to ROS
        sensor_msgs::PointCloud2 final_msg;
        pcl::toROSMsg(*cloud_final, final_msg);
        final_msg.header.stamp = ros::Time::now();
        final_msg.header.frame_id = output_frame_;
        return final_msg;
    }

    bool processCallback(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res)
    {
        ROS_INFO("Waiting point cloud message for [%fs]...", waiting_time);

        sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(input_topic_, ros::Duration(waiting_time));
        if (msg)
        {
            pub_.publish(pointCloudFilter(msg));

            res.success = true;
            res.message = "Point cloud received and published.";
            ROS_INFO("Processed and published point cloud.");
        }
        else
        {
            res.success = false;
            res.message = "Timeout: No point cloud received.";
            ROS_WARN("No point cloud received within timeout.");
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_relay_node");
    ros::NodeHandle nh("~"); // use private namespace for parameters
    PointCloudRelay relay(nh);
    ros::spin();
    return 0;
}
