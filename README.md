FTSensor class
==================

License
-------

This program is released under GPLv2, see LICENSE.txt for details.

Build
-----

Typical cmake building:

    mkdir build (please use build in lowercase, the ros packages are not still completely integrated with standard cmake files, and hence the relative path is case-sensitive)
    cd build
    cmake-gui ..   CONFIGURE ROS_PKGS OPTIONS, AND IT IS SUGGESTED TO INSTALL IN A LOCAL PATH
    click configure
    click regenerate
    close cmake-gui
    make

ToDo
----

- Proper documentation
- Write some examples
- ROS visualization of sensor and data
