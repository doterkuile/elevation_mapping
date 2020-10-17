#include "elevation_mapping/PlaneDetection.h"


PlaneDetection::PlaneDetection(ros::NodeHandle &nh)
    :
      nh_(nh),
      extract_(false)

{
    pubPolygon_ = nh_.advertise<custom_planner_msgs::polygonArray>("Obstacle/polygons", 20);
    this->setupRegionGrowing();
    this->setupPlaneSegmentation();
    this->setupNormalCalc();

}


PlaneDetection::PlaneDetection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud, ros::NodeHandle &nh)
    :
      nh_(nh),
      extract_(false)
{

    this->setupRegionGrowing();
    this->setupPlaneSegmentation();
    this->setupNormalCalc();


}

void PlaneDetection::setSubscriber(const std::string& subString)
{
    sub_ = nh_.subscribe(subString, 1, &PlaneDetection::cloudCallBack, this);
}

void PlaneDetection::cloudCallBack(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud)
{
    this->setCloud(inputCloud);

}

void PlaneDetection::setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud)
{

    cloud_ = inputCloud;
    rollArray_.resize(cloud_->points.size(), 0.0);
    pitchArray_.resize(cloud_->points.size(), 0.0);
    cloudNormals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>> ();
    pcl::Normal pointNormal(0.0,0.0,0.0);
    cloudNormals_->insert(cloudNormals_->begin(), cloud_->size(), pointNormal);

    std::string sensorOrigin;

    bool useTorsoLidar;

    nh_.param("use_torso_lidar", useTorsoLidar, true);

    if(useTorsoLidar)
    {
        nh_.param("sensor_frame_id_torso", sensorOrigin, std::string("rgbd_torso_optical_frame"));

    }
    else
    {
        nh_.param("sensor_frame_id", sensorOrigin, std::string("rgbd_optical_frame"));

    }


    //  if(!this->changeCloudFrame(cloud_, tfListener_,tfBuffer_, "odom"))
    if(cloud_->header.frame_id != "/odom")
    {
        ROS_ERROR_STREAM("<<<< cloud frame id != odom, obstacles not published");
        return;
    }



    if(!this->setSensorOrigin(cloud_, tfListener_, tfBuffer_, sensorOrigin))
    {
        ROS_ERROR_STREAM("Could not locate sensor origin, obstacles not published");
        return;
    }

    this->extractGround(cloud_);

    if(!(obstacleCloud_->points.size() == 0))
    {


        bool useIntegralNormalMethod;
        nh_.param("normal_use_integral",useIntegralNormalMethod, false);

        if(useIntegralNormalMethod)
        {

            if(!cloud_->isOrganized())
            {
                ROS_WARN("Cloud is not organized, unable to use integral image normal calculation");
                ROS_WARN("Obstacles not published");
                return;
            }

            this->getIntegralNormals(cloud_);

        }
        else
        {
            this->getNormalVectors(obstacleCloud_);

        }
    }

    this->segmentPlanes();

    this->normalVectorMarker();

}


bool PlaneDetection::getPitchRoll(const int &index, float &roll, float &pitch)
{

    roll = rollArray_[index];
    pitch = pitchArray_[index];

    return 0;

}

void PlaneDetection::extractGround(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud)
{

    // Set up segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundPlane = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
    pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
    obstacleIndices_ = boost::make_shared<pcl::PointIndices>();
    pcl::PointIndices::Ptr cloudIndices = boost::make_shared<pcl::PointIndices>();

    obstacleCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::copyPointCloud(*inputCloud,*obstacleCloud_);

    std::vector<int> indices(inputCloud->size());
    std::iota (indices.begin(), indices.end(), 0);
    std::vector<int> indices2 = indices;

    obstacleIndices_->indices = indices;//boost::make_shared<pcl::PointIndices>(indices);
    cloudIndices->indices = indices2;

    while(obstacleCloud_->size() > planeCloudPercentage_ * cloud_->size())
    {
        // Segment the largest planar component from the remaining cloud
        seg_.setInputCloud(obstacleCloud_);
        seg_.segment(*inliers, *coefficients);

        if(inliers->indices.size() == 0)
        {

            return;
        }


        pcl::Normal planeNormal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);

        int inliersSize = inliers->indices.size();
        for( int ii{0}; ii < inliersSize; ii++)
        {
            cloudNormals_->points[cloudIndices->indices[inliers->indices[ii]]] = planeNormal;
        }

        //
        // Extract the Obstacles
        extract_.setInputCloud(obstacleCloud_);
        extract_.setIndices(inliers);
        extract_.setNegative(false);
        extract_.filter(*groundPlane);
        groundPlane->header = cloud_->header;
        this->pubPlaneCloud(groundPlane);


        extract_.setNegative(true);
        extract_.filter(obstacleIndices_->indices);
        extract_.filter(*obstacleCloud_);

        obstacleCloud_->header = cloud_->header;
        this->pubPlaneCloud(obstacleCloud_);

        int obstacleSize = obstacleIndices_->indices.size();
        for(int ii{0}; ii < obstacleSize; ii++)
        {
            obstacleIndices_->indices[ii] = cloudIndices->indices[obstacleIndices_->indices[ii]];
        }
        cloudIndices->indices = obstacleIndices_->indices;
    }



}




void PlaneDetection::pubCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pubPointCloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                                                                         "/Output/ObstacleCloud", 1);

    pubPointCloud_.publish(*cloud);
}

void PlaneDetection::pubPlaneCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pubPlaneCloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                                                                         "/Output/planeCloud", 1);

    pubPlaneCloud_.publish(*cloud);
}


bool PlaneDetection::changeCloudFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,  std::shared_ptr<tf2_ros::TransformListener> &tfListener, tf2_ros::Buffer &tfBuffer, const std::string &targetFrame)
{
    geometry_msgs::TransformStamped transformStamped;

    tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    try{

        transformStamped = tfBuffer.lookupTransform( targetFrame, pointCloud->header.frame_id, ros::Time(0));
        ROS_INFO_STREAM(">>> succesfully found frame transform");

    }
    // Catch if base_link cannot be found
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ROS_WARN("Obstacles not published");
        ros::Duration(1.0).sleep();
        return 0;
    }
    Eigen::Quaternionf q;
    Eigen::Vector3f t;

    // Set rotation
    q.x() = transformStamped.transform.rotation.x;
    q.y() = transformStamped.transform.rotation.y;
    q.z() = transformStamped.transform.rotation.z;
    q.w() = transformStamped.transform.rotation.w;

    // Set translation
    t.x() = transformStamped.transform.translation.x;
    t.y() = transformStamped.transform.translation.y;
    t.z() = transformStamped.transform.translation.z;

    pcl::transformPointCloud(*pointCloud, *pointCloud,
                             t, q);
    pointCloud->header.frame_id = "odom";

    return 1;
}

bool PlaneDetection::setSensorOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,  std::shared_ptr<tf2_ros::TransformListener> &tfListener, tf2_ros::Buffer &tfBuffer, const std::string &sensorOrigin)
{
    geometry_msgs::TransformStamped transformStamped;

    tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    try{

        transformStamped = tfBuffer.lookupTransform( "odom", sensorOrigin, ros::Time(0), ros::Duration(5.0));

    }
    // Catch if base_link cannot be found
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ROS_WARN("Obstacles not published");
        return 0;
    }
    // Set the sensor origin to rgbd_optical_frame
    pointCloud->sensor_origin_.x() = transformStamped.transform.translation.x;
    pointCloud->sensor_origin_.y() = transformStamped.transform.translation.y;
    pointCloud->sensor_origin_.z() = transformStamped.transform.translation.z;

    return 1;

}

std::vector<ObstaclePlane> PlaneDetection::getPlaneArray()
{
    return planeArray_;
}


void PlaneDetection::getNormalVectors(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloudTemp = boost::make_shared<pcl::PointCloud<pcl::Normal>> ();
    // Set input cloud
    ne_.setInputCloud(inputCloud);
    // Calculate Normal Vectors
    ne_.compute(*cloudTemp);
    int obstacleSize = obstacleIndices_->indices.size();
    for(int ii{0}; ii < obstacleSize;  ii++)
    {
        cloudNormals_->points[obstacleIndices_->indices[ii]] = cloudTemp->points[ii];
    }


}

void PlaneDetection::getIntegralNormals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud)
{
    cloudNormals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>> ();
    pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>> ();


    nie_.setInputCloud(inputCloud);
    nie_.useSensorOriginAsViewPoint();
    nie_.compute(*normals);

    // Extract normals of obstacles
    pcl::ExtractIndices<pcl::Normal> extractNormals;
    extractNormals.setInputCloud(normals);
    extractNormals.setIndices(obstacleIndices_);
    extractNormals.setNegative(false);
    extractNormals.filter(*cloudNormals_);

}

bool PlaneDetection::validateObstacle(const pcl::PointIndices::Ptr &indices, pcl::Normal &planeNormal)
{
    // Calculate normal vector of plane

    pcl::ExtractIndices<pcl::Normal> extract;
    pcl::PointCloud<pcl::Normal>::Ptr planeNormals = boost::make_shared< pcl::PointCloud<pcl::Normal> > ();
    pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices> ();

    planeNormal._Normal::normal_x = 0;
    planeNormal._Normal::normal_y = 0;
    planeNormal._Normal::normal_z = 0;


    extract.setInputCloud(cloudNormals_);
    extract.setIndices(indices);
    extract.filter(*planeNormals);

    int jj{0};

    int normalVectorSize = planeNormals->points.size();
    for(int ii{0}; ii < normalVectorSize; ii++)
    {
        if(std::isnan(planeNormals->points[ii]._Normal::normal_x)||std::isnan(planeNormals->points[ii]._Normal::normal_y)||std::isnan(planeNormals->points[ii]._Normal::normal_z))
        {
            continue;
        }
        planeNormal._Normal::normal_x += planeNormals->points[ii]._Normal::normal_x;
        planeNormal._Normal::normal_y += planeNormals->points[ii]._Normal::normal_y;
        planeNormal._Normal::normal_z += planeNormals->points[ii]._Normal::normal_z;
        jj++;
    }

    planeNormal._Normal::normal_x = planeNormal._Normal::normal_x/jj;
    planeNormal._Normal::normal_y =  planeNormal._Normal::normal_y/jj;
    planeNormal._Normal::normal_z =  planeNormal._Normal::normal_z/jj;

    // Check if it's a relevant plane
    if(std::abs(planeNormal._Normal::normal_z) >= std::cos(M_PI / 2.5))
    {
        return 1;
    }
    else
    {
        return 0;
    }

}


void PlaneDetection::segmentPlanes()
{
    planeArray_.clear();
    polygonArray_.polygonArray.clear();

    // Region growing
    // Initialize required variables for regionGrowing
    std::vector<pcl::PointIndices> clusters;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // set Obstacle cloud and corresponding normals as input
    reg_.setInputCloud(cloud_);
    reg_.setInputNormals(cloudNormals_);
    // Extract clusters from the regionGrowing algorithm
    reg_.extract(clusters);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColored = reg_.getColoredCloud ();
    cloudColored->header.frame_id = cloud_->header.frame_id;
    int counter{0};

    extract_.setInputCloud(cloud_);
    extract_.setNegative(false);
    // Loop through clusters and save them in obstacle array
    while (counter < static_cast<int>(clusters.size()))
    {
        pcl::PointIndices::Ptr clusterIndices = boost::make_shared<pcl::PointIndices> (clusters[counter]);

        // set right indices
        extract_.setIndices(clusterIndices);
        // filter out points
        extract_.filter(*cloudCluster);

        pcl::Normal planeNormal;
        if(this->validateObstacle(clusterIndices, planeNormal))
        {
            ObstaclePlane obstacle(cloud_, planeNormal, clusters[counter]);
            planeArray_.push_back(obstacle);
            polygonArray_.polygonArray.push_back(obstacle.getPolygon());

            int clusterSize = clusterIndices->indices.size();
            for(int ii{0}; ii < clusterSize; ii++)
            {

                rollArray_[clusterIndices->indices[ii]] = obstacle.getRoll();
                pitchArray_[clusterIndices->indices[ii]] = obstacle.getPitch();

            }


        }
        else
        {

        }

        counter++;
    }

    this->pubCloud(cloudColored);
    pubPolygon_.publish(polygonArray_);
}


void PlaneDetection::normalVectorMarker()
{
    markerArray_.markers.clear();
    int planeArraySize = planeArray_.size();
    for(int ii{0}; ii < planeArraySize; ii++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";

        marker.header.stamp = ros::Time::now();
        marker.ns = "normalVector";
        marker.id = ii;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.scale.x = 0.5;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        pcl::PointXYZRGB point;
        Eigen::Quaterniond q;
        Eigen::Vector3d normalVector;



        planeArray_[ii].getCenterPoint(point);
        planeArray_[ii].getNormalVector(normalVector);
        q.setFromTwoVectors(Eigen::Vector3d::UnitX(), normalVector);
        //    planeArray_[ii].getOrientation(q);
        q.normalize();

        marker.pose.position.x = point._PointXYZRGB::x;
        marker.pose.position.y = point._PointXYZRGB::y;
        marker.pose.position.z = point._PointXYZRGB::z;

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();


        markerArray_.markers.push_back(marker);
    }

    pubMarkerArray_ = nh_.advertise<visualization_msgs::MarkerArray>(
                "/Output/NormalVectors", 20);
    pubMarkerArray_.publish(markerArray_);



}

void PlaneDetection::setupPlaneSegmentation()
{
    double distanceThreshold;
    int maxIterations;
    nh_.param("plane_distance_threshold", distanceThreshold, 0.01);
    nh_.param("plane_cloud_percentage", planeCloudPercentage_, 0.7);

    nh_.param("plane_max_iterations", maxIterations, 1000);
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setDistanceThreshold(distanceThreshold);
    seg_.setMaxIterations(maxIterations);
}

void PlaneDetection::setupRegionGrowing()
{
    int minClusterSize;
    int maxClusterSize;
    int neighbours;
    double smoothnessThreshold;
    double curvatureThreshold;
    nh_.param("region_min_cluster_size", minClusterSize, 100);
    nh_.param("region_max_cluster_size", maxClusterSize, 10000000);
    nh_.param("region_neighbours", neighbours, 10);
    nh_.param("region_smoothness_threshold", smoothnessThreshold, 1.0);
    nh_.param("region_curvature_threshold", curvatureThreshold, 1.0);
    std::cout << "Min cluster size = " << minClusterSize << '\n';

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    reg_.setMinClusterSize(minClusterSize);
    reg_.setMaxClusterSize(maxClusterSize);
    reg_.setSearchMethod(tree);
    reg_.setNumberOfNeighbours(neighbours);
    reg_.setSmoothnessThreshold(smoothnessThreshold /
                                180.0 * M_PI);
    reg_.setCurvatureThreshold(curvatureThreshold);

}


void PlaneDetection::setupNormalCalc()
{



    float normalMaxDepthChange;
    float normalSmoothingSize;
    nh_.param("normal_max_depth", normalMaxDepthChange, 0.01f);
    nh_.param("normal_smoothing_size", normalSmoothingSize, 5.0f);

    std::vector<pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::NormalEstimationMethod> estimationMethodArray;

    estimationMethodArray.push_back(nie_.COVARIANCE_MATRIX);
    estimationMethodArray.push_back(nie_.AVERAGE_3D_GRADIENT);
    estimationMethodArray.push_back(nie_.AVERAGE_DEPTH_CHANGE);
    estimationMethodArray.push_back(nie_.SIMPLE_3D_GRADIENT);

    int estimationMethod;

    nh_.param("normal_estimation_method", estimationMethod, 0);
    nie_.setNormalEstimationMethod (estimationMethodArray[estimationMethod]);
    nie_.setBorderPolicy(nie_.BORDER_POLICY_MIRROR);
    nie_.setMaxDepthChangeFactor(normalMaxDepthChange);
    nie_.setNormalSmoothingSize(normalSmoothingSize);



    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZRGB>());
    double searchRadius;
    int kNeighbours;
    bool normalNN;
    nh_.param("normal_nearest_neighbour", normalNN, false);
    nh_.param("normal_search_radius", searchRadius, 0.01);
    nh_.param("normal_k_neighbours", kNeighbours,20);


    ne_.setSearchMethod(tree);
    if(normalNN)
    {
        ne_.setKSearch(kNeighbours);

        std::cout << "nearest_neighbours = " << kNeighbours << '\n';
    }else
    {
        ne_.setRadiusSearch(searchRadius);
        std::cout << "nearest_neighbours = " << normalNN << '\n';

    }
    ne_.useSensorOriginAsViewPoint();
    ne_.setNumberOfThreads(0);

}
