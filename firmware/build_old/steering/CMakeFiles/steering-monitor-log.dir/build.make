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

# Utility rule file for steering-monitor-log.

# Include the progress variables for this target.
include steering/CMakeFiles/steering-monitor-log.dir/progress.make

steering/CMakeFiles/steering-monitor-log:
	cd /home/user/oscc/firmware/build/steering && screen -L /dev/ttyACM0 115200

steering-monitor-log: steering/CMakeFiles/steering-monitor-log
steering-monitor-log: steering/CMakeFiles/steering-monitor-log.dir/build.make

.PHONY : steering-monitor-log

# Rule to build all files generated by this target.
steering/CMakeFiles/steering-monitor-log.dir/build: steering-monitor-log

.PHONY : steering/CMakeFiles/steering-monitor-log.dir/build

steering/CMakeFiles/steering-monitor-log.dir/clean:
	cd /home/user/oscc/firmware/build/steering && $(CMAKE_COMMAND) -P CMakeFiles/steering-monitor-log.dir/cmake_clean.cmake
.PHONY : steering/CMakeFiles/steering-monitor-log.dir/clean

steering/CMakeFiles/steering-monitor-log.dir/depend:
	cd /home/user/oscc/firmware/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/oscc/firmware /home/user/oscc/firmware/steering /home/user/oscc/firmware/build /home/user/oscc/firmware/build/steering /home/user/oscc/firmware/build/steering/CMakeFiles/steering-monitor-log.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : steering/CMakeFiles/steering-monitor-log.dir/depend

