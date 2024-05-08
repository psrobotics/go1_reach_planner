[![CI](https://github.com/IMRCLab/libmotioncapture/actions/workflows/CI.yml/badge.svg)](https://github.com/IMRCLab/libmotioncapture/actions/workflows/CI.yml)

# libmotioncapture
Interface Abstraction for Motion Capture System APIs such as VICON, OptiTrack, Qualisys, Nokov, FZMotion, or VRPN.

This can be used as C++ library or Python package. For Python, use

```
pip install motioncapture
```

For C++, follow the instructions below.

This is a fork of https://github.com/USC-ACTLab/libmotioncapture/ with the following changes:

- Python bindings
- Factory method
- Refactored API
- Support for VRPN by default

## Compile options
By default, `libmotioncapture` supports the following hardware:

- VICON - SDK git submodule
- Qualisys - SDK git submodule
- OptiTrack - binary parsing over network (no dependency)
- VRPN - SDK git submodule
- NOKOV - manually obtain SDK and copy to deps/nokov_sdk/ and copy the .so file to the /lib or /usr/lib.
- FZMotion - no dependency
- Motion Analysis - manually obtain SDK and copy to deps/cortex_sdk_linux/

CMake flags can be used to disable individual systems in `CMakeLists.txt`.

## Prerequisites

```
sudo apt install libboost-system-dev libboost-thread-dev libeigen3-dev ninja-build
```

## C++

```
git submodule init
git submodule update
mkdir build
cd build
cmake ..
make
```

An example application is in `examples/main.cpp`. Run it using

```
./motioncapture_example <mocap type> <ip address>
```

## Python (Development)

```
git submodule init
git submodule update
python3 setup.py develop --user
python3 examples/python.py
```

# v3.8.6
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: Go1

not support robot: Laikago, B1, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Dependencies
* [Unitree](https://www.unitree.com/download)
```bash
Legged_sport    >= v1.36.0
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [g++](https://gcc.gnu.org/) (version 8.3.0 or higher)


### Build
```bash
mkdir build
cd build
cmake ..
make
```

If you want to build the python wrapper, then replace the cmake line with:
```bash
cmake -DPYTHON_BUILD=TRUE ..
```

If can not find pybind11 headers, then add
```bash
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/pybind11/include)
```
at line 14 in python_wrapper/CMakeLists.txt.

If can not find msgpack.hpp, then
```bash
sudo apt install libmsgpack*
```

### Run

#### Cpp
Run examples with 'sudo' for memory locking.

#### Python
##### arm
change `sys.path.append('../lib/python/amd64')` to `sys.path.append('../lib/python/arm64')`


### Run the vicon node
```
cd build
sudo ./motioncapture_example vicon 192.168.1.39
```

```
sudo ./rplidar_publisher --channel --serial /dev/ttyUSB0 256000
```
