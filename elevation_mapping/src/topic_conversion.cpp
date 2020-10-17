

#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <nav_msgs/Odometry.h>
//#include "elevation_mapping/obstacleplane.h"
//#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pubCloud;
ros::Publisher pubPose;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_conversion");
  ros::NodeHandle nh("~");
  PlaneDetection planeArray(nh);
  ROS_INFO_STREAM("Starting node");
  std::string pointCloudTopic;
  nh.param("point_cloud_topic", pointCloudTopic, std::string("/rgbd/depth/points"));

  //pubPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1000);



  while(ros::ok())
  {
    planeArray.setSubscriber(pointCloudTopic);

    ros::spinOnce();



//  // Spin
//  ros::AsyncSpinner spinner(1); // Use n threads
//  spinner.start();
  }
//  ros::waitForShutdown();
  return 0;
}
