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
CMAKE_SOURCE_DIR = /home/neofelis/VRX/drift

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/neofelis/VRX/drift/build

# Include any dependencies generated for this target.
include tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/depend.make

# Include the progress variables for this target.
include tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/progress.make

# Include the compile flags for this target's objects.
include tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/flags.make

tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.o: tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/flags.make
tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.o: ../tests/cheetahkinematics_test/cheetahkinematics_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/neofelis/VRX/drift/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.o"
	cd /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.o -c /home/neofelis/VRX/drift/tests/cheetahkinematics_test/cheetahkinematics_test.cpp

tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.i"
	cd /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/neofelis/VRX/drift/tests/cheetahkinematics_test/cheetahkinematics_test.cpp > CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.i

tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.s"
	cd /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/neofelis/VRX/drift/tests/cheetahkinematics_test/cheetahkinematics_test.cpp -o CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.s

# Object files for target TEST_cheetahkinematics_test
TEST_cheetahkinematics_test_OBJECTS = \
"CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.o"

# External object files for target TEST_cheetahkinematics_test
TEST_cheetahkinematics_test_EXTERNAL_OBJECTS =

tests/cheetahkinematics_test/TEST_cheetahkinematics_test: tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/cheetahkinematics_test.cpp.o
tests/cheetahkinematics_test/TEST_cheetahkinematics_test: tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/build.make
tests/cheetahkinematics_test/TEST_cheetahkinematics_test: lib/libgtest_main.a
tests/cheetahkinematics_test/TEST_cheetahkinematics_test: libdrift.a
tests/cheetahkinematics_test/TEST_cheetahkinematics_test: lib/libgtest.a
tests/cheetahkinematics_test/TEST_cheetahkinematics_test: tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/neofelis/VRX/drift/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TEST_cheetahkinematics_test"
	cd /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TEST_cheetahkinematics_test.dir/link.txt --verbose=$(VERBOSE)
	cd /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test && /usr/bin/cmake -D TEST_TARGET=TEST_cheetahkinematics_test -D TEST_EXECUTABLE=/home/neofelis/VRX/drift/build/tests/cheetahkinematics_test/TEST_cheetahkinematics_test -D TEST_EXECUTOR= -D TEST_WORKING_DIR=/home/neofelis/VRX/drift/build/tests/cheetahkinematics_test -D TEST_EXTRA_ARGS= -D TEST_PROPERTIES= -D TEST_PREFIX= -D TEST_SUFFIX= -D NO_PRETTY_TYPES=FALSE -D NO_PRETTY_VALUES=FALSE -D TEST_LIST=TEST_cheetahkinematics_test_TESTS -D CTEST_FILE=/home/neofelis/VRX/drift/build/tests/cheetahkinematics_test/TEST_cheetahkinematics_test[1]_tests.cmake -D TEST_DISCOVERY_TIMEOUT=5 -P /usr/share/cmake-3.16/Modules/GoogleTestAddTests.cmake

# Rule to build all files generated by this target.
tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/build: tests/cheetahkinematics_test/TEST_cheetahkinematics_test

.PHONY : tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/build

tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/clean:
	cd /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test && $(CMAKE_COMMAND) -P CMakeFiles/TEST_cheetahkinematics_test.dir/cmake_clean.cmake
.PHONY : tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/clean

tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/depend:
	cd /home/neofelis/VRX/drift/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/neofelis/VRX/drift /home/neofelis/VRX/drift/tests/cheetahkinematics_test /home/neofelis/VRX/drift/build /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test /home/neofelis/VRX/drift/build/tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/cheetahkinematics_test/CMakeFiles/TEST_cheetahkinematics_test.dir/depend
