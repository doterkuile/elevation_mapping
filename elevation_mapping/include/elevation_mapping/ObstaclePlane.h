#ifndef OBSTACLEPLANE_H
#define OBSTACLEPLANE_H

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <math_utils/math_utils.h>




class ObstaclePlane
{
public:
  ObstaclePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &plane, pcl::Normal &planeNormal);
  ObstaclePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, pcl::Normal &planeNormal, const pcl::PointIndices &indices);

  float getRoll();
  float getPitch();
  void setEulerAngles();
  bool atPosition(const Eigen::Vector2d &pos);
  void getCenterPoint(pcl::PointXYZRGB &point);
  void setOrientation();
  void getOrientation(Eigen::Quaterniond &q);
  void setPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);
  void getNormalVector(Eigen::Vector3d &normalVector);
  void setPolygon();
  geometry_msgs::PolygonStamped getPolygon();


  pcl::PointIndices obstacleIndices_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_;

private:
  pcl::Normal planeNormal_;
  grid_map::Polygon polygon_;

  float roll_;
  float pitch_;
  Eigen::Quaterniond orientation_;



};



#endif // OBSTACLEPLANE_H
