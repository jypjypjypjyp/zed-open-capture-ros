# zed-open-capture-ros
## Introduction
A simple ros driver for ZED using zed-open-capture which doesn't depend on CUDA
## Dependencies

* ros (Kinetic/Melodic/Noetic)
* [zed-open-capture](https://github.com/stereolabs/zed-open-capture)

## Usage

Complie:
``` bash
catkin_make
```

Run:
``` bash
source devel/setup.bash
roslaunch zed-open-capture-ros zed_node.launch
```


