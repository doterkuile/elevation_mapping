#ifndef PLANEDETECTION_H
#define PLANEDETECTION_H

// ElevationMapping
#include "elevation_mapping/ObstaclePlane.h"
#include "custom_planner_msgs/polygonArray.h"
// Vector
#include <vector>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/cloud_iterator.h>


// TF
#include <tf2_ros/transform_listener.h>

// MarkerArray
#include <visualization_msgs/MarkerArray.h>

class PlaneDetection
{
public:

  // Basic constructor
  PlaneDetection(ros::NodeHandle &nh);

  // Construct class and set point cloud
  PlaneDetection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, ros::NodeHandle &nh);

  // Get array of obstacle planes
  std::vector<ObstaclePlane> getPlaneArray();

  // Get roll and pitch of cloud at given index
  bool getPitchRoll(const int &index, float &roll, float &pitch);


  // Set point cloud
  void setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud);


private:

  // Set up parameters for for plane segmentation
  void setupPlaneSegmentation();
  void setupRegionGrowing();
  void setupNormalCalc();

  // Cloud callback for point cloud
  void cloudCallBack(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud);

  // Set subscriber
  void setSubscriber(const std::string& subString);

  // Extract largest plane with plane segmentation
  void extractGround(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud);

  // Create obstacle plane array
  void segmentPlanes();

  // Publish point cloud
  void pubCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  // Publish single plane of point cloud
  void pubPlaneCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  // Change the frame of point cloud, return true if succeeds
  bool changeCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,  std::shared_ptr<tf2_ros::TransformListener> &tfListener, tf2_ros::Buffer &tfBuffer_, const std::string &targetFrame);

  // Set the origin of the sensor of the point cloud, return true if succeeds
  bool setSensorOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,  std::shared_ptr<tf2_ros::TransformListener> &tfListener, tf2_ros::Buffer &tfBuffer_, const std::string &s);

  // calculate the normal vectors of the pointcloud
  void getNormalVectors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud);
  // calculate normal vector with integral image
  void getIntegralNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud);

  // Calculate average normal vector of obstacle plane, return false if too vertical
  bool validateObstacle(const pcl::PointIndices::Ptr &indices, pcl::Normal &planeNormal);


  // Publish normal vector as arrows of each obstacle plane
  void normalVectorMarker();





  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  std::vector<ObstaclePlane> planeArray_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacleCloud_;
  pcl::PointIndices::Ptr obstacleIndices_;
  std::vector<float> rollArray_;
  std::vector<float> pitchArray_;

  // Segmentation settings
  pcl::SACSegmentation<pcl::PointXYZRGB> seg_;
  // Normal estimator
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne_;
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> nie_;
  custom_planner_msgs::polygonArray polygonArray_;
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals_;

  // Region Growing Settings
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg_;

  // Extract Indices settings
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher  pubPointCloud_;
  ros::Publisher  pubPlaneCloud_;

  ros::Publisher  pubMarkerArray_;
  ros::Publisher  pubPolygon_;


  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  tf2_ros::Buffer tfBuffer_;
  visualization_msgs::MarkerArray markerArray_;
  double planeCloudPercentage_;

};

#endif // PLANEDETECTION_H
