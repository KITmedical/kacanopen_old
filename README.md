# KaCanOpen

KaCanOpen is an easy-to-use [CanOpen](https://en.wikipedia.org/wiki/CANopen) stack, which consists of  four parts:

* __Drivers:__ A wide range of hardware is supported using different CAN drivers. They have been developed by the [CanFestival project](http://www.canfestival.org/). Read [this](drivers/README_DRIVERS.md) for details.

* __Core:__ This is a library which implements basic CANOpen protocols like [NMT](https://en.wikipedia.org/wiki/CANopen#Network_management_.28NMT.29_protocols), [SDO](https://en.wikipedia.org/wiki/CANopen#Service_Data_Object_.28SDO.29_protocol) and [PDO](https://en.wikipedia.org/wiki/CANopen#Process_Data_Object_.28PDO.29_protocol). As an example, you can easily fetch a value from a device (*uploading* in CanOpen terminology) via `core.sdo.upload(node_id, index, subindex)`. It furthermore allows you to register callbacks on certain events or any incoming messages, so one can build arbitrary CanOpen nodes (master or slave) using this library.

* __Master:__ This library is intended to be used for a master node. It detects all nodes in a network and allows to access them via standardized [CiA® profiles](http://www.can-cia.org/can-knowledge/canopen/canopen-profiles/). For example, on a motor device (profile CiA® 402) you can simply call `mymotor.set_entry("Target velocity", 500)`. A main feature of this library is transparent SDO/PDO access to dictionary entries: By default a value is fetched and set via SDO. But you can configure PDO mappings to instead keep the value up-to-date in background via (more lightweight) PDO messages. The call itself (`mymotor.set_entry("Target velocity", 500)`) keeps unchanged.

* __ROS Bridge:__ This library provides a bridge to a [ROS](http://www.ros.org/) network, which makes KaCanOpen especially interesting for Robotics. After setting up the CanOpen network, the library can publish slave nodes so they are accessible through ROS messages. Special effort is put into the use of standardized message types which allows interaction with other software from the ROS universe. For example, motors can be operated using JointState messages.

KaCanOpen is designed to make use of modern C++11/14 features and to avoid redundant code on the user side wherever possible. This makes the interface neater than it is in other products on the market.

## Quick start

First make sure you've got a recent C++ compiler with C++14 support ([GCC](https://gcc.gnu.org/) >= 4.9, [Clang](http://clang.llvm.org/) >= 3.5), as well as [CMake](https://cmake.org/) >= 3.2 and [Boost](http://www.boost.org/) >= 1.46.1.

KaCanOpen without the ROS part can be built easily using CMake:

~~~bash
git clone git@gitlab.ira.uka.de:thomaskeh/kacanopen.git
cd kacanopen
mkdir build
cd build
cmake -DCAN_DRIVER_NAME=<driver> -DNO_ROS=On ..
make
~~~

`<driver>` can be one of the following: serial, socket, virtual, lincan, peak\_linux.

The `examples` directory lists some examples on how to use KaCanOpen libraries. You can run them from the `build/examples` directory.

KaCanOpen including the ROS part must be built using [Catkin](http://wiki.ros.org/catkin/Tutorials). Make sure you have [ROS Jade](http://www.ros.org/install/) installed. Go to your [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and clone the repository into `src`:

~~~bash
cd your_catkin_workspace/src
git clone git@gitlab.ira.uka.de:thomaskeh/kacanopen.git
cd ..
catkin_make -DCAN_DRIVER_NAME=<driver>
~~~

When building with Catkin, you can excute example programs like that:

~~~bash
cd your_catkin_workspace
source devel/setup.bash
./devel/lib/kacanopen/kacanopen_example_ros
~~~

Complete build instructions can be found [here](doc/Installation.md).

## Documentation

Full documentation can be found at [https://thk1.github.io/kacanopen/](https://thk1.github.io/kacanopen/).

## License

Core, Master and ROS Bridge are licensed under the [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause) license. Drivers from [CanFestival](http://www.canfestival.org/) are licensed under the [LGPLv2.1+](https://opensource.org/licenses/LGPL-2.1) license.

