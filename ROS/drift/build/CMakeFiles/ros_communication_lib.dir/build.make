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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/neofelis/VRX/drift/ROS/drift

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/neofelis/VRX/drift/ROS/drift/build

# Include any dependencies generated for this target.
include CMakeFiles/ros_communication_lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ros_communication_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ros_communication_lib.dir/flags.make

CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.o: CMakeFiles/ros_communication_lib.dir/flags.make
CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.o: ../src/communication/ros_subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/neofelis/VRX/drift/ROS/drift/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.o -c /home/neofelis/VRX/drift/ROS/drift/src/communication/ros_subscriber.cpp

CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/neofelis/VRX/drift/ROS/drift/src/communication/ros_subscriber.cpp > CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.i

CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/neofelis/VRX/drift/ROS/drift/src/communication/ros_subscriber.cpp -o CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.s

CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.o: CMakeFiles/ros_communication_lib.dir/flags.make
CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.o: ../src/communication/ros_publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/neofelis/VRX/drift/ROS/drift/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.o -c /home/neofelis/VRX/drift/ROS/drift/src/communication/ros_publisher.cpp

CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/neofelis/VRX/drift/ROS/drift/src/communication/ros_publisher.cpp > CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.i

CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/neofelis/VRX/drift/ROS/drift/src/communication/ros_publisher.cpp -o CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.s

# Object files for target ros_communication_lib
ros_communication_lib_OBJECTS = \
"CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.o" \
"CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.o"

# External object files for target ros_communication_lib
ros_communication_lib_EXTERNAL_OBJECTS =

../lib/libros_communication_lib.so: CMakeFiles/ros_communication_lib.dir/src/communication/ros_subscriber.cpp.o
../lib/libros_communication_lib.so: CMakeFiles/ros_communication_lib.dir/src/communication/ros_publisher.cpp.o
../lib/libros_communication_lib.so: CMakeFiles/ros_communication_lib.dir/build.make
../lib/libros_communication_lib.so: CMakeFiles/ros_communication_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/neofelis/VRX/drift/ROS/drift/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library ../lib/libros_communication_lib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ros_communication_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ros_communication_lib.dir/build: ../lib/libros_communication_lib.so

.PHONY : CMakeFiles/ros_communication_lib.dir/build

CMakeFiles/ros_communication_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ros_communication_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ros_communication_lib.dir/clean

CMakeFiles/ros_communication_lib.dir/depend:
	cd /home/neofelis/VRX/drift/ROS/drift/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/neofelis/VRX/drift/ROS/drift /home/neofelis/VRX/drift/ROS/drift /home/neofelis/VRX/drift/ROS/drift/build /home/neofelis/VRX/drift/ROS/drift/build /home/neofelis/VRX/drift/ROS/drift/build/CMakeFiles/ros_communication_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ros_communication_lib.dir/depend
