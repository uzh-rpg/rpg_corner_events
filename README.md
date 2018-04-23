# Fast Event-based Corner Detection
Inspired by frame-based pre-processing techniques that reduce an image to a set of features, which are typically the input to higher-level algorithms, we propose a method to reduce an event stream to a *corner event* stream.
Our goal is twofold: extract relevant tracking information (corners do not suffer from the aperture problem) and decrease the event rate for later processing stages.
Our event-based corner detector is very efficient due to its design principle, which consists of working on the Surface of Active Events (a map with the timestamp of the latest event at each pixel) using only comparison operations.
Our method asynchronously processes event by event with very low latency.
Our implementation is capable of processing a million events per second on a single core (less than a micro-second per event) and reduces the event rate by a factor of 10 to 20.

![corners_screenshot](https://user-images.githubusercontent.com/670994/33448948-9fd5c3fa-d607-11e7-8be3-d57fa93391ea.png)
Left: image with *all* events, right: image with only corner events. Event color depicts polarity (i.e., the sign of the brightness change).

[![YouTube video](http://rpg.ifi.uzh.ch/img/papers/BMVC17_Mueggler_thumb_play.jpg)](http://www.youtube.com/watch?v=tgvM4ELesgI)

This code also contains the Spatially-Adaptive Harris Method used for comparison.
For more details, please read our [BMVC'17 paper](http://rpg.ifi.uzh.ch/docs/BMVC17_Mueggler.pdf) or have look at the [poster](http://rpg.ifi.uzh.ch/docs/BMVC17_Mueggler_poster.pdf).

## Publication

If you use this code in an academic context, please cite the following [BMVC'17 publication](http://rpg.ifi.uzh.ch/docs/BMVC17_Mueggler.pdf):

E. Mueggler, C. Bartolozzi, D. Scaramuzza:
**Fast Event-based Corner Detection.**
British Machine Vision Conference (BMVC), London, 2017.

    @inproceedings{Mueggler17BMVC,
     author = {Mueggler, Elias and Bartolozzi, Chiara and Scaramuzza, Davide},
     title = {Fast Event-based Corner Detection},
     booktitle = {British Machine Vision Conference (BMVC)},
     year = {2017}
    }

# Disclaimer and License
This code has been tested with ROS kinetic on Ubuntu 16.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
The source code is released under a GNU General Public License (GPL).
For a commercial license, please contact the [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html)

# Instructions
## Installation
1. Install the [DVS/DAVIS ROS driver](https://github.com/uzh-rpg/rpg_dvs_ros) (you only need the `dvs_msgs` and `dvs_renderer` packages).
2. Clone the repository to your ROS workspace
    ```
    git clone https://github.com/uzh-rpg/rpg_corner_events
    ```
3. Build it using the following command:
    ```
    roscd corner_event_detector
    catkin build --this
    ```

## Using a Dataset
To get a bag file from the [Event-Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html):
```
wget http://rpg.ifi.uzh.ch/datasets/davis/shapes_6dof.bag
```

Run the detector and visualization launch file:
```
roslaunch corner_event_detector bag.launch
```

In a separate terminal, run a bag file, e.g.:
```
rosbag play shapes_6dof.bag
```

## Using the DAVIS Event Camera (Live Mode)
Please run the file:
```
roslaunch corner_event_detector davis_live.launch
```

