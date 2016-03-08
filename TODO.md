# Roadmap

This is a yet incomplete list of features which are planned for the future. Feel free to contact me you have any further ideas or if you want to contribute code.

## Short-term

* __Bridge:__ Implement a ROS service for accessing any dictionary entry of a slave by name.
* __Bridge:__ Subscribers / Publishers: Allow configuration of queue_size.
* __Core:__ Automatically map PDOs like they are configured in slave's dictionary.
* __Core:__ Add methods for manipulating PDO mapping configuration in slave's dictionary.
* __Core:__ Add ability to remove/change existing PDO transmitters / receivers / mappings.
* __Project:__ Fix/Improve Doxygen mainpage.
* __Master:__ Add profile specific convenience methods (using Lambda expressions). For example, CiA 402 control word manipulation methods are indicated.

## Long-term

* __Master:__ Make master a CiA 301 compliant node. This means it has to implement some slave functionality. Changes in core library are necessary in order to parse SDO request messages.
* __Project:__ Implement a slave node library.
* __Project:__ Check thread safety. Should be achievable with very few changes (mutexes on driver and callback vectors).
* __Bridge:__ Have a look into ros_control. Can we use it for controlling CiA 402 devices?
* __Core:__ Support multiple device profiles per slave.
* __Core:__ Improve error handling.
* __Core:__ Implement missing protocol parts like LSS, EMERGENCY and SYNC master.

