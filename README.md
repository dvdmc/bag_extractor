# Bag Extractor

## Overview

This is a package created to extract raw data from rosbags. Noticing that there is a lack of a bag extractor using the [rosbag API](https://wiki.ros.org/rosbag/Code%20API), I implemented nodes to extract different types of common information as images, imu, velodyne pointclouds or gps.

This method is faster and more robust than classical approaches based on playing the rosbag file (e.g. [export images](https://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data) or [exporting data to CSV](https://answers.ros.org/question/9102/how-to-extract-data-from-bag/)), as it isn't dynamic and ensures no information overflow (solved by reducing the speed of the rosbag play).

For simplicity, the code is repeated through the nodes. In the future, parent abstract classes will be implemented in order to ease the code organisation.
Creating a new node for your own extraction is really easy. The only changes you will have to do are in the rosmsgs to extract, the extract method and the method to process each message.

The different implemented nodes in this repo offer a wide variety of examples:

* img2jpg: saved in different files with a sequence number and timestamp. It uses OpenCV as external library and [cv_bridge](https://wiki.ros.org/cv_bridge).

* imu2txt: saved in a single file including the timestamp and data per line. The most simple example.

* pc2bin: saved in different files with a sequence number and timestamp being these binary. It uses a [PointCloud iterator](https://wiki.ros.org/pcl/Overview).

* gps2txt: saved in a single file including the tiemestamp and data per line. It subscribes to multiple topics and uses [Novatel GPS driver](https://github.com/swri-robotics/novatel_gps_driver) package.

## Installation

### Building from source

#### Dependencies

* [Robot Operating System (ROS)](http://wiki.ros.org/).

* [OpenCV](https://opencv.org/) 3.2. (The specific version is due to incompatibility with cv_bridge).

* ROS dependencies:
    * roscpp
    * std_msgs
    * sensor_msgs
    * rosbag
    * cv_bridge
    * novatel_gps_driver (modified)

* [fmt](https://github.com/fmtlib/fmt): Used for ease in formatting text.

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using:

    cd catkin_workspace/src
    git clone https://github.com/davidmorillacabello/husky_highlevel_controller.git
    cd ../
    catkin build

## Usage

Check the launchfile to create one that adjusts to your specific needs in structure, sensors and topics. A minimal example is included as a test and can be run with:

    roslaunch bag_extractor extract_dataset.launch\ 
        bag:="[...]/catkin_ws/src/bag_extractor/test/test.bag"

## Launch files

* `extract_dataset.launch:` launch all the sensor nodes for the test.bag file. This is an specific use case for the dataset showed in README_dataset.md but can be usde as an example for other cases.

## Nodes

All the nodes share the same parameters:

* `bag:` name of the rosbag. (Notice that in case of the .launch file the path is relative to ~/.ros or $ROS_HOME directory).

* `topic:` name of the topic to extract the data. (gps2txt contains an example for multi-topic extraction).

* `folder:` name of the folder to store the data. This folder must exist prior to the execution of the node. The .launch file includes the execution of a script to iteratively create two level directories.

Optional:

* `start_time:` start time to extract the data from the bag.

* `end_time:` end time to extract the data from the bag.

### img2jpg

Extract images from a compressed (!!!) image topic to a jpg image using cv_bridge and OpenCV. The images are saved with the following name convention:

    *deviceName_timestamp.jpg*

### imu2txt


Extract IMU data from a imu topic to a data.txt file. Each message is stored in one line following the next convention:

*timestamp roll pitch yaw ang_vel_X ang_vel_Y ang_vel_Z lin_acc_X lin_acc_Y lin_acc_Z*

### pc2bin


Extract PointCloud data from a Velodyne Points topic binary files. Each message is stored in one binary file as a Nx4 matrix (being N the number of points per message) following the next convention (used in the [KITTI](http://www.cvlibs.net/datasets/kitti/) dataset):

*x y z intensity*

The point clouds are saved with the following name convention:

    *deviceName_timestamp.bin*

In order to open and parse the data from the binary file, the followings commands are examples of use:

* `numpy:`

```python
    points = np.fromfile(filename, dtype=np.float32).reshape(-1, 4)
```

* `MATLAB:`

```matlab
    fileID = fopen(filename, 'r');
    format = 'float32';
    data = fread(fileID, Inf, format);
    xyzi=reshape(data,4,length(data)/4)';
```

### gps2txt


Extract GPS data from the novatel_driver GPGGA and GPRMC topics to a data.txt file. Each message is stored in one line following the next convention:

*timestamp NMEA_standard_message* ([GPGGA](https://docs.novatel.com/oem7/Content/Logs/GPGGA.htm) or [GPRMC](https://docs.novatel.com/oem7/Content/Logs/GPRMC.htm))

## Sequence extractor

In case that your data has a similar format than the result of this extractor, you might also be interested in extracting sub-sequences from the extracted ones.
With this goal, I created another tool stored in this [repository](https://github.com/davdmc/extract_sequence). 
Details about the extraction of sub-sequences are explained in the repository.