#!/bin/bash


pushd packages
mv micro_ros-humble-gcc-10/SConscript micro_ros-humble-gcc-10/SConscript.bak 
mv micro_ros_rtthread_component/SConscript.bak micro_ros_rtthread_component/SConscript
popd
scons -c && rm -rf packages/micro_ros_rtthread_component/builder/libmicroros/ && scons --build_microros

set -e
pushd packages
find micro_ros-humble-gcc-10  -name "libmicroros.a" -exec cp -raf micro_ros_rtthread_component/builder/libmicroros/libmicroros.a   {} \;
mv micro_ros-humble-gcc-10/SConscript.bak  micro_ros-humble-gcc-10/SConscript
mv micro_ros_rtthread_component/SConscript  micro_ros_rtthread_component/SConscript.bak
popd

