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
CMAKE_SOURCE_DIR = /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build

# Include any dependencies generated for this target.
include omnicopter_controller/CMakeFiles/attitude_controller.dir/depend.make

# Include the progress variables for this target.
include omnicopter_controller/CMakeFiles/attitude_controller.dir/progress.make

# Include the compile flags for this target's objects.
include omnicopter_controller/CMakeFiles/attitude_controller.dir/flags.make

# Object files for target attitude_controller
attitude_controller_OBJECTS =

# External object files for target attitude_controller
attitude_controller_EXTERNAL_OBJECTS =

/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/devel/lib/libattitude_controller.so: omnicopter_controller/CMakeFiles/attitude_controller.dir/build.make
/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/devel/lib/libattitude_controller.so: omnicopter_controller/CMakeFiles/attitude_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking C++ shared library /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/devel/lib/libattitude_controller.so"
	cd /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/omnicopter_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/attitude_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
omnicopter_controller/CMakeFiles/attitude_controller.dir/build: /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/devel/lib/libattitude_controller.so

.PHONY : omnicopter_controller/CMakeFiles/attitude_controller.dir/build

omnicopter_controller/CMakeFiles/attitude_controller.dir/requires:

.PHONY : omnicopter_controller/CMakeFiles/attitude_controller.dir/requires

omnicopter_controller/CMakeFiles/attitude_controller.dir/clean:
	cd /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/omnicopter_controller && $(CMAKE_COMMAND) -P CMakeFiles/attitude_controller.dir/cmake_clean.cmake
.PHONY : omnicopter_controller/CMakeFiles/attitude_controller.dir/clean

omnicopter_controller/CMakeFiles/attitude_controller.dir/depend:
	cd /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/src /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/src/omnicopter_controller /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/omnicopter_controller /home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/omnicopter_controller/CMakeFiles/attitude_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : omnicopter_controller/CMakeFiles/attitude_controller.dir/depend

