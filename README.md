# table_mapping 
Table mapping provide robot the ability to build semantic map of tables around his working environment.

##Prerequisites:
1. soma_io - a package of [SOMA](https://github.com/anosnowhong/soma/tree/soma2) is used to record all kinds of ROS message to MongoDB for the later process.
2. strands_morse - a robot simulator to run and test your application.

PCL and OpenCV is used to implement the following pcakges.

##control_morse
control_morse act as an core progrom for the table_mapping package, it controls the PTU hardware and send the navigation goal.
##Features:
* Free space search, sending navigation goal automatically.
* PTU movement control

##table_detection
###Features:
* Extract possible table surface(Plane RANSAC method is used).
* Call registration service provided by table_registration to get a larger table plane.
* Two representations of table convex and concave hull are store in DB.
* Merge and Classify tables from one region(overlap)
* Change detection implemented base on table history stored in DB using statistical method.
* Visualization tools(launch file) for table stored in MongoDB, view table_detection::Table type msg.

##table_registration
##Features:
* Register two given point cloud (two point cloud better have overlap area in global space).
* Call table_deteciton service to extract table plane for new registered clouds.
* Visualization tools(launch file) for table stored in MongoDB, view sensor::PointCloud2 type msg.

#Usage
To start a whole experiment:
```
roscd control_morse/launch
#This script starts morse simulation, Rviz, Navigation, control_morse core, table detection and registration service.
./start_morse 
```
