# KaCanOpen

KaCanOpen is an easy-to-use CanOpen stack, which consists of the following four parts:

* __Drivers:__ Contains various CAN drivers for supported hardware. They have been developed by the [CanFestival project](http://www.canfestival.org/). See [canfestival_drivers/README.md](canfestival_drivers/README.md) for details.

* __Core:__ This is a library which implements basic CANOpen protocols like NMT, SDO and PDO. For example, you can upload sth. (in CanOpen terminology, this means fetching a value from a device) very easily via `core.sdo.upload(node_id, index, subindex)`. It furthermore allows you to register callbacks on certain events or any incoming messages, so one can build arbitrary CanOpen nodes (master or slave) using this library.

* __Master:__ This library is intended to be used for a master node. It detects all nodes in a network and allows to access them via standardized [CiA® profiles](http://www.can-cia.org/can-knowledge/canopen/canopen-profiles/). For example, on a motor device (profile CiA® 402) you can simply call `mymotor.set_entry("Target velocity", 500)`. A main feature of this library is transparent SDO/PDO access to dictionary entries: By default a value is fetched and set via SDO. But you can configure PDO mappings to instead keep the value up-to-date in background via (more lightweight) PDO messages, while the programmer still uses the same methods (like `mymotor.set_entry("Target velocity", 500)`).

* __ROS Bridge:__ This library provides a bridge to a ROS network, which makes KaCanOpen especially interesting for Robotics. After setting up the CanOpen network, the library can publish slave nodes so they are accessible through ROS messages. Special effort is put into the use of standardized message types which allows interaction with other software from the ROS universe. For example motors can be operated using JointState messages.

KaCanOpen is designed to make use of modern C++11/14 features and to avoid redundant code on the user side wherever possible. This makes the interface neater than it is in other products on the market.

__KaCanOpen is still under active development and not yet recommended for production use!__

## Build and Run

KaCanOpen without the ROS part can be built easily using CMake:

```bash
mkdir build
cd build
cmake -DCAN_DRIVER_NAME=<driver> -DNO_ROS=On ..
make
```

where `<driver>` can be one of the following: _lincan, peak\_linux, serial, socket, virtual_. The requirements are a C++ compiler with C++14 support (tested with GCC 4.9 and Clang 3.5) and CMake >= 3.2. The _peak\_linux_ driver also needs installed [PCAN drivers](http://www.peak-system.com/fileadmin/media/linux/index.htm).

The `examples` directory lists some examples on how to use KaCanOpen libraries. You can run them from the `build/examples` directory.

KaCanOpen including the ROS part must be built using [Catkin](http://wiki.ros.org/catkin/Tutorials):

```bash
# init workspace if you don't have one yet
mkdir ~/arbitrary_path/catkin_ws
cd ~/arbitrary_path/catkin_ws
mkdir src
cd src
catkin_init_workspace

# link the KaCanOpen code or clone the repository directly into ~/arbitrary_path/catkin_ws/src
ln -s /path_to_your_KaCanOpen_repository/ ./kacanopen
cd ..

# build the project using catkin
# catkin_make will forward arguments to CMake
catkin_make -DCAN_DRIVER_NAME=<driver>
```

You can run the given example programs like that:

```bash
cd ~/arbitrary_path/catkin_ws
source install/setup.bash
./install/bin/kacanopen_example_ros
```


## License

Core, Profiles and ROS parts are licensed under the [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause) license. The drivers from [CanFestival](http://www.canfestival.org/) are licensed under the [LGPLv2.1+](https://opensource.org/licenses/LGPL-2.1) license. See [COPYING.txt](COPYING.txt) for details.
