#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>

class PointCloudRelay
{
public:
    PointCloudRelay(ros::NodeHandle &nh) : nh_(nh)
    {
        // Parameters
        nh_.param<std::string>("input_topic", input_topic_, "/input_pointcloud");
        nh_.param<std::string>("output_topic", output_topic_, "/output_pointcloud");
        nh_.param<std::string>("output_frame", output_frame_, "modified_frame");

        // Setup publisher and service clent
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        srv_ = nh_.advertiseService("/pointcloud_relay/capture_pointcloud", &PointCloudRelay::processCallback, this);

        ROS_INFO("PointCloudRelay initialized. Listening on [%s], publishing on [%s]", input_topic_.c_str(), output_topic_.c_str());
    }

private:
    const double waiting_time = 5.0;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::ServiceServer srv_;
    std::string input_topic_, output_topic_, output_frame_;

    sensor_msgs::PointCloud2 pointCloudFilter(const sensor_msgs::PointCloud2ConstPtr &msg)
    {

        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);

        /* Modify pointcloud here

        */

        // Convert back to ROS message
        sensor_msgs::PointCloud2 modified_msg;
        pcl::toROSMsg(cloud, modified_msg);

        // Modify header
        modified_msg.header.stamp = ros::Time::now();
        modified_msg.header.frame_id = output_frame_;

        return modified_msg;
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
