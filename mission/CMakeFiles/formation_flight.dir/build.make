# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/amov/Prometheus/Modules/mission

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amov/Prometheus/build/mission

# Include any dependencies generated for this target.
include CMakeFiles/formation_flight.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/formation_flight.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/formation_flight.dir/flags.make

CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o: CMakeFiles/formation_flight.dir/flags.make
CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o: /home/amov/Prometheus/Modules/mission/formation_flight/formation_flight.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amov/Prometheus/build/mission/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o -c /home/amov/Prometheus/Modules/mission/formation_flight/formation_flight.cpp

CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amov/Prometheus/Modules/mission/formation_flight/formation_flight.cpp > CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.i

CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amov/Prometheus/Modules/mission/formation_flight/formation_flight.cpp -o CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.s

CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.requires:

.PHONY : CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.requires

CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.provides: CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.requires
	$(MAKE) -f CMakeFiles/formation_flight.dir/build.make CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.provides.build
.PHONY : CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.provides

CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.provides.build: CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o


# Object files for target formation_flight
formation_flight_OBJECTS = \
"CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o"

# External object files for target formation_flight
formation_flight_EXTERNAL_OBJECTS =

/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: CMakeFiles/formation_flight.dir/build.make
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libmavros.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libGeographic.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libeigen_conversions.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libmavconn.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libclass_loader.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/libPocoFoundation.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libdl.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libroslib.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/librospack.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libtf2_ros.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libactionlib.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libmessage_filters.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libroscpp.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/librosconsole.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libtf2.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/librostime.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /opt/ros/melodic/lib/libcpp_common.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight: CMakeFiles/formation_flight.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amov/Prometheus/build/mission/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/formation_flight.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/formation_flight.dir/build: /home/amov/Prometheus/devel/lib/prometheus_mission/formation_flight

.PHONY : CMakeFiles/formation_flight.dir/build

CMakeFiles/formation_flight.dir/requires: CMakeFiles/formation_flight.dir/formation_flight/formation_flight.cpp.o.requires

.PHONY : CMakeFiles/formation_flight.dir/requires

CMakeFiles/formation_flight.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/formation_flight.dir/cmake_clean.cmake
.PHONY : CMakeFiles/formation_flight.dir/clean

CMakeFiles/formation_flight.dir/depend:
	cd /home/amov/Prometheus/build/mission && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amov/Prometheus/Modules/mission /home/amov/Prometheus/Modules/mission /home/amov/Prometheus/build/mission /home/amov/Prometheus/build/mission /home/amov/Prometheus/build/mission/CMakeFiles/formation_flight.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/formation_flight.dir/depend

