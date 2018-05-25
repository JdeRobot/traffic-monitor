# traffic-monitor
Traffic monitor is a vision based traffic sensor, able to gather several traffic stats using a simple stationary calibrated camera. TM is able
to process traffic flow on multiple lanes simultaneously in real-time. In the current version the software is able to:
* Estimate the velocity of the vehicles
* Classify the vehicles into five categories: Motorcycle, Car, Van, Bus and Truck
* Vehicle count

[In this video](https://youtu.be/5Nu_BTPTc94) we can see the tool working on an offiline video of highway with multiple lanes.

![Alt text](doc/traffic-monitor.png?raw=true "Some examples of traffic-monitor classification")

### 1. Related Papers

If you find this code useful in your research, please consider citing:

        @article{doi: 10.1117/1.JEI.25.3.033021,
        author = { Redouane Kachach, José María Cañas},
        title = {Hybrid three-dimensional and support vector machine approach for automatic vehicle tracking and classification using a single camera},
        journal = {Journal of Electronic Imaging},
        volume = {25},
        number = {},
        pages = {25 - 25 - 24},
        year = {2016},
        doi = {10.1117/1.JEI.25.3.033021},
        URL = {https://doi.org/10.1117/1.JEI.25.3.033021},
        eprint = {}
        }


### 2. Installation

git clone https://github.com/rkachach/traffic-monitor

#### 2.1 Requiered Dependencies

sudo apt-get install libopencv-dev libcairomm-1.0-dev libgtkmm-3.0-dev libgsl-dev

#### 2.3 Build

     cd traffic-monitor
     mkdir build
     cd build
     cmake ../src/
     make -j

### 3 Configuration

Traffic monitor uses two configuration files: **trafficmonitor.cfg** and **camera.cfg**. The first one is the main configuration file and
it contains a set of key=value per each line. This file is read during the initialization and is updated each time the user saves
the program configuration (Save button).

The second one is the camera configuration/calibration file and uses the following syntax (metric values are in millimeter):

        #extrinsics, position
        positionX 0
        positionY 0
        positionZ 10302
        positionH 1

        #orientation
        FOApositionX -476
        FOApositionY 50577
        FOApositionZ 0
        FOApositionH 1
        roll 0

        #Intrensics
        fx 1143
        fy 1143
        skew 0
        u0 288
        v0 360
        columns 720
        rows 576

In addition to this files the traffic monitor needs access to the GUI file **trafficmonitor.glade** and the SVM model file **svm.yml**.
All these files are located in the cfg directory.

### 4 Usage

    cd build
    ln -s ../cfg/trafficMonitor.cfg trafficmonitor.cfg
    ./trafficmonitor path_to_video_file
    ./trafficmonitor ../traffic-videos/video-0042-o-4.MPG (i.e)

### 5 traffic-monitor dataset

As part of this work we created a database with more than 100 traffic videos. This dataset is available for [download](http://jderobot.org/store/trafficmonitor-dataset/) for
acaedmic non-comercial use. Video's naming uses the following syntax: **video-#id-[o|i]-#lanes.ext** where o/i stands for outgoing and ingoing videos. Stats of the different
videos ara available on the [excel sheet](traffic-monitor-video-database.xlsx). If you find this dataset useful please consider citing the traffic-monitor [paper](#related-papers).

### 6 License

  Traffic monitor is released under a GPLv3 license. For a closed-source version of this software for commercial use, please contact the author(s).
