# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/user/oscc/firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/oscc/firmware/build

# Utility rule file for can-gateway-monitor-log.

# Include the progress variables for this target.
include can_gateway/CMakeFiles/can-gateway-monitor-log.dir/progress.make

can_gateway/CMakeFiles/can-gateway-monitor-log:
	cd /home/user/oscc/firmware/build/can_gateway && screen -L /dev/ttyACM0 115200

can-gateway-monitor-log: can_gateway/CMakeFiles/can-gateway-monitor-log
can-gateway-monitor-log: can_gateway/CMakeFiles/can-gateway-monitor-log.dir/build.make

.PHONY : can-gateway-monitor-log

# Rule to build all files generated by this target.
can_gateway/CMakeFiles/can-gateway-monitor-log.dir/build: can-gateway-monitor-log

.PHONY : can_gateway/CMakeFiles/can-gateway-monitor-log.dir/build

can_gateway/CMakeFiles/can-gateway-monitor-log.dir/clean:
	cd /home/user/oscc/firmware/build/can_gateway && $(CMAKE_COMMAND) -P CMakeFiles/can-gateway-monitor-log.dir/cmake_clean.cmake
.PHONY : can_gateway/CMakeFiles/can-gateway-monitor-log.dir/clean

can_gateway/CMakeFiles/can-gateway-monitor-log.dir/depend:
	cd /home/user/oscc/firmware/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/oscc/firmware /home/user/oscc/firmware/can_gateway /home/user/oscc/firmware/build /home/user/oscc/firmware/build/can_gateway /home/user/oscc/firmware/build/can_gateway/CMakeFiles/can-gateway-monitor-log.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : can_gateway/CMakeFiles/can-gateway-monitor-log.dir/depend

