#include "elevation_mapping/ObstaclePlane.h"




ObstaclePlane::ObstaclePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &plane, pcl::Normal &planeNormal)
   : plane_(plane),
     planeNormal_(planeNormal)
{
  this->setOrientation();

  this->setEulerAngles();


}

ObstaclePlane::ObstaclePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, pcl::Normal &planeNormal, const pcl::PointIndices &indices)
   :planeNormal_(planeNormal),
    obstacleIndices_(indices)
{
  this->setPlane(pointCloud);

  this->setPolygon();

  this->setOrientation();

  this->setEulerAngles();


}

void ObstaclePlane::setPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud)
{
  pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>(obstacleIndices_);
  plane_ = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(pointCloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane_);


}

void ObstaclePlane::setPolygon()
{

   // Create a Concave Hull representation of the projected inliers
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::ConvexHull<pcl::PointXYZRGB> chull;
   chull.setInputCloud (plane_);
//   chull.setAlpha (0.1);
   chull.reconstruct (*cloud_hull);


   std::vector<grid_map::Position> vertices(cloud_hull->points.size());

   for(int ii{0}; ii < cloud_hull->points.size(); ii++)
   {
     vertices[ii].x() = cloud_hull->points[ii]._PointXYZRGB::x;
     vertices[ii].y() = cloud_hull->points[ii]._PointXYZRGB::y;
     polygon_.addVertex(vertices[ii]);
   }
   polygon_.setFrameId(plane_->header.frame_id);

}

void ObstaclePlane::setOrientation()
{
  Eigen::Vector3d normalVector;
  normalVector.x() = planeNormal_._Normal::normal_x;
  normalVector.y() = planeNormal_._Normal::normal_y;
  normalVector.z() = planeNormal_._Normal::normal_z;


  // If normalvector containts NaNs, set to Z unit vector
  if(std::isnan(normalVector.sum()))
  {
    normalVector = Eigen::Vector3d::UnitZ();
  }

  orientation_.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normalVector);
  orientation_.normalize();

}

void ObstaclePlane::setEulerAngles()
{
  Eigen::Vector3d rpy;
  extractRollPitchYaw(orientation_, &rpy.x(),&rpy.y(), &rpy.z());

  roll_ = static_cast<float>(rpy.x());
  pitch_ = static_cast<float>(rpy.y());
}

float ObstaclePlane::getRoll()
{
  return roll_;
}

float ObstaclePlane::getPitch()
{
  return pitch_;
}

void ObstaclePlane::getOrientation( Eigen::Quaterniond &q)
{
    q = orientation_;
}


void ObstaclePlane::getCenterPoint(pcl::PointXYZRGB &point)
{
  pcl::computeCentroid(*plane_, point);
}

void ObstaclePlane::getNormalVector(Eigen::Vector3d &normalVector)
{
  normalVector.x() = planeNormal_._Normal::normal_x;
  normalVector.y() = planeNormal_._Normal::normal_y;
  normalVector.z() = planeNormal_._Normal::normal_z;
}

geometry_msgs::PolygonStamped ObstaclePlane::getPolygon()
{
  geometry_msgs::PolygonStamped polStamped;
  grid_map::PolygonRosConverter::toMessage(polygon_, polStamped);
  return polStamped;
}




