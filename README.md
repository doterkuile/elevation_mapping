# Elevation map package

This package is an extension of the [elevation_mapping package] (https://github.com/ANYbotics/elevation_mapping) of Anybotics. It maps the environment to gridmap containing values for the height. In the addition to the already existing height layer there are two extra layers added to the grid map containing the roll and the pitch of the environment. This is published as the elevation map.
![gazebo_world](./elevation_mapping_demos/doc/gazebo_world.png#center)
## Plane detection

To add roll and pitch to the grid map the class [PlaneDetection](.elevation_mapping/src/PlaneDetection.cpp) extracts the roll and pitch of the given pointcloud. This is done in three steps:

### Extract biggest plane

To speed up the process the biggest plane will be determined using [pcl::SACSegmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html?highlight=plane%20segementation). Paramaters for the algorithm can be tweaked in [talos_robot.yaml](.elevation_mapping_demos/config/robots/talos_robot.yaml). This algorithm gives the plane equation which easily converts to the planes normal vector.

### Calculate normal vectors

The normal vectors of the rest of the cloud are calculated with [pcl::NormalEstimation](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html?highlight=normal). Paramaters for the algorithm can be tweaked in [talos_robot.yaml](.elevation_mapping_demos/config/robots/talos_robot.yaml).

### Region growing

With the previous algorithms the normal vector of each point in the pointcloud is calculated. With [pcl::RegionGrowing](https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html?highlight=region%20growing) different planes are extracted. Paramaters for the algorithm can be tweaked in [talos_robot.yaml](.elevation_mapping_demos/config/robots/talos_robot.yaml). The topic */Output/planeCloud* shows the different planes in different colors. Points of the pointcloud that are not part of any plane according to the algorithm are displayed in red and get a pitch and roll value of pi/2. 
![Plane cloud](./elevation_mapping_demos/doc/plane_cloud.png#center)
### Elevation map
The values for height, roll and pitch are added to the elevation map. Note that only the topic */elevation_mapping/elevation_map_raw* contains the values for pitch and roll. The map fusion still needs to be fixed such that the topic */elevation_mapping/elevation_map_fused* also contains the orientation.
![Elevation map](./elevation_mapping_demos/doc/elevation_map.png#center)

