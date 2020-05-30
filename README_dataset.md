# ISA Rescue dataset



Paper:

## Overview

The dataset is classified in different folders by year, day and sequence. 
Each sequence has a name corresponding to its content. Inside each sequence 
there are different directories for each "device" (i.e. rgb image, imu, ...).

The different available devices are:

* RGB camera.

* Thermal camera.

* Inertial Measurement Unit (IMU).

* LiDAR Velodyne.

* Global Positioning System (GPS).

* GPS with Real-time Kinematic correction (RTK).

In every case the timestamp is the ROS time recorded in the original rosbag file for that message.

## Images

The dataset contains two sets of images. One corresponds to the RGB spectrum and the other to the infrarred spectrum.
The name convention followed is:

    device_timestamp.jpg

The sequence number starts at 0 for each sequence.

Both images are stored in different directories named:

    image_rgb/

    image_the/

The capture ratio is 25 fps for both cameras. The image sizes are 704x576 pixels for both cameras.  

## IMU

The data from the IMU is stored in a single "data.txt" file inside `imu/ ` directory for each sequence. The file stores the messages as successive lines.
The structure for each line is:

    timestamp roll pitch yaw ang_vel_X ang_vel_Y ang_vel_Z lin_acc_X lin_acc_Y lin_acc_Z

- Roll, pitch and yaw are in rad and about fixed axes XYZ.

- Angular velocities (ang_vel_{X, Y, Z}) are in rad/sec.

- Linear accelerations (lin_acc_{X, Y, Z}) are in m/s².

The capture ratio is 64 Hz.

## Velodyne points

Each message is stored in one binary file as a Nx4 matrix (being N the number of points per message) following the next convention:

    x y z intensity

- x, y and z are the position respect the Velodyne sensor in meters.

- The intensity offers information about the energy of the reflected laser pulse. This enables to differenciate between different materials in the scene.

They are stored in the folder `velodyne_points/`. The name convention followed is:

    device_timestamp.jpg

## GPS

The dataset contains two gps data. `gps_user/` includes the normal measure from the GPS. `gps_rtk/` contains the corrected data using Real-time Kinematic correction from a ground base station.

The data is stored in a "data.txt" file inside each directory. The file stores the messages as succesive lines.
The structure for each line is:

    timestamp GPS_NMEA_data

The GPS NMEA data provided are standard GPGGA and GPRMC messages. Further information about their contents can be checked in the links below alongside an example for each message:

([GPGGA](https://docs.novatel.com/oem7/Content/Logs/GPGGA.htm)) 

$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,
AAAA*60

([GPRMC](https://docs.novatel.com/oem7/Content/Logs/GPRMC.htm))

$GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20

## Sequence extractor

The following [repository](https://github.com/davdmc/extract_sequence) offers tools to extract sub-sequences from each sequence. This can be used to separate data for an specific purpose (e.g. scenes with/without movement only, scenes with/without a semantic class as people or cars ...).

Detailed information about the usage of the tools are included in the repository. Two scripts can be used:

- `extract_single.py:` script to extract data from a single sequence. It recieves several options (see extract_single_options.py).

- `extract_file.py:` script to automate the process of extracting data from multiple sequences. It uses a .txt file as a single parameter with a specific format to describe the different sequences that can be shared and edited (see [Extraction format](https://github.com/davdmc/extract_sequence#extraction-format) or [extraction_format.txt](https://github.com/davdmc/extract_sequence/blob/master/extraction_format.txt)).

### Specific rules for extraction format

In the case of this dataset the extraction format for `extract_file` must follow these rules:

- `folder:` it corresponds to the specific sequence under the year/day/sequence path convention.

- `devices and formats:` the devices with their correspondent formats are listed below. The number of devices to extract is optional. The user may choose which devices to extract depending on the usage of the extracted sequence. If the device is assigned a wrong data format the data will be extracted wrong or an error will appear.

    |      Device     |   Data format  |
    |:---------------:|:--------------:|
    |    image_rgb    | multiple_files |
    |    image_the    | multiple_files |
    |       imu       |   single_file  |
    | velodyne_points | multiple_files |
    |     gps_user    |   single_file  |
    |     gps_rtk     |   single_file  |

- `intervals:` must be in the same time units as the correspondent in the file names or "data.txt" files.