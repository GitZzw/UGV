# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/uav/CLion-2019.2.4/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/uav/CLion-2019.2.4/clion-2019.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/uav/lzy_ws/src/offb_posctl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uav/lzy_ws/src/offb_posctl/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/offb_enable_vicon.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/offb_enable_vicon.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/offb_enable_vicon.dir/flags.make

CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.o: CMakeFiles/offb_enable_vicon.dir/flags.make
CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.o: ../src/offb_enable_vicon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uav/lzy_ws/src/offb_posctl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.o -c /home/uav/lzy_ws/src/offb_posctl/src/offb_enable_vicon.cpp

CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uav/lzy_ws/src/offb_posctl/src/offb_enable_vicon.cpp > CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.i

CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uav/lzy_ws/src/offb_posctl/src/offb_enable_vicon.cpp -o CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.s

# Object files for target offb_enable_vicon
offb_enable_vicon_OBJECTS = \
"CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.o"

# External object files for target offb_enable_vicon
offb_enable_vicon_EXTERNAL_OBJECTS =

devel/lib/offb_posctl/offb_enable_vicon: CMakeFiles/offb_enable_vicon.dir/src/offb_enable_vicon.cpp.o
devel/lib/offb_posctl/offb_enable_vicon: CMakeFiles/offb_enable_vicon.dir/build.make
devel/lib/offb_posctl/offb_enable_vicon: /home/uav/mavros_ws/devel/.private/mavros/lib/libmavros.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libGeographic.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libeigen_conversions.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
devel/lib/offb_posctl/offb_enable_vicon: /home/uav/mavros_ws/devel/.private/libmavconn/lib/libmavconn.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/libPocoFoundation.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libroslib.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/librospack.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libtf2_ros.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libactionlib.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libtf2.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libroscpp.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/librosconsole.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/librostime.so
devel/lib/offb_posctl/offb_enable_vicon: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/offb_posctl/offb_enable_vicon: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/offb_posctl/offb_enable_vicon: CMakeFiles/offb_enable_vicon.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/uav/lzy_ws/src/offb_posctl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/offb_posctl/offb_enable_vicon"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/offb_enable_vicon.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/offb_enable_vicon.dir/build: devel/lib/offb_posctl/offb_enable_vicon

.PHONY : CMakeFiles/offb_enable_vicon.dir/build

CMakeFiles/offb_enable_vicon.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/offb_enable_vicon.dir/cmake_clean.cmake
.PHONY : CMakeFiles/offb_enable_vicon.dir/clean

CMakeFiles/offb_enable_vicon.dir/depend:
	cd /home/uav/lzy_ws/src/offb_posctl/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uav/lzy_ws/src/offb_posctl /home/uav/lzy_ws/src/offb_posctl /home/uav/lzy_ws/src/offb_posctl/cmake-build-debug /home/uav/lzy_ws/src/offb_posctl/cmake-build-debug /home/uav/lzy_ws/src/offb_posctl/cmake-build-debug/CMakeFiles/offb_enable_vicon.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/offb_enable_vicon.dir/depend
