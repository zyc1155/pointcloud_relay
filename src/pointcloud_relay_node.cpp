#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_relay/SetRun.h>

class PointCloudRelay
{
public:
  PointCloudRelay(ros::NodeHandle& nh)
  {
    // Parameters
    nh.param<std::string>("input_topic", input_topic_, "/input_pointcloud");
    nh.param<std::string>("output_topic", output_topic_, "/output_pointcloud");
    nh.param<std::string>("output_frame", output_frame_, "modified_frame");

    // Setup subscribers and publishers
    sub_ = nh.subscribe(input_topic_, 1, &PointCloudRelay::pointCloudCallback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    srv_ = nh.advertiseService("/set_run", &PointCloudRelay::setRunCallback, this);

    run_ = true;
    ROS_INFO("PointCloudRelay initialized. Listening on [%s], publishing on [%s]", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceServer srv_;
  bool run_;
  std::string input_topic_, output_topic_, output_frame_;

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!run_)
      return;

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

    pub_.publish(modified_msg);
  }

  bool setRunCallback(pointcloud_relay::SetRun::Request& req,
                      pointcloud_relay::SetRun::Response& res)
  {
    run_ = req.run;
    ROS_INFO("Set run = %s", run_ ? "true" : "false");
    res.success = true;
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_relay_node");
  ros::NodeHandle nh("~"); // use private namespace for parameters
  PointCloudRelay relay(nh);
  ros::spin();
  return 0;
}
