# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/cody/Downloads/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/cody/Downloads/clion-2019.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cody/catkin_ws/src/quadrotor_sim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cody/catkin_ws/build

# Include any dependencies generated for this target.
include CMakeFiles/move_quad_s.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/move_quad_s.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/move_quad_s.dir/flags.make

CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.o: CMakeFiles/move_quad_s.dir/flags.make
CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.o: /home/cody/catkin_ws/src/quadrotor_sim/src/move_quad_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cody/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.o -c /home/cody/catkin_ws/src/quadrotor_sim/src/move_quad_server.cpp

CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cody/catkin_ws/src/quadrotor_sim/src/move_quad_server.cpp > CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.i

CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cody/catkin_ws/src/quadrotor_sim/src/move_quad_server.cpp -o CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.s

# Object files for target move_quad_s
move_quad_s_OBJECTS = \
"CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.o"

# External object files for target move_quad_s
move_quad_s_EXTERNAL_OBJECTS =

/home/cody/catkin_ws/devel/lib/quadrotor_sim/move_quad_s: CMakeFiles/move_quad_s.dir/src/move_quad_server.cpp.o
/home/cody/catkin_ws/devel/lib/quadrotor_sim/move_quad_s: CMakeFiles/move_quad_s.dir/build.make
/home/cody/catkin_ws/devel/lib/quadrotor_sim/move_quad_s: CMakeFiles/move_quad_s.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cody/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/cody/catkin_ws/devel/lib/quadrotor_sim/move_quad_s"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/move_quad_s.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/move_quad_s.dir/build: /home/cody/catkin_ws/devel/lib/quadrotor_sim/move_quad_s

.PHONY : CMakeFiles/move_quad_s.dir/build

CMakeFiles/move_quad_s.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/move_quad_s.dir/cmake_clean.cmake
.PHONY : CMakeFiles/move_quad_s.dir/clean

CMakeFiles/move_quad_s.dir/depend:
	cd /home/cody/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cody/catkin_ws/src/quadrotor_sim /home/cody/catkin_ws/src/quadrotor_sim /home/cody/catkin_ws/build /home/cody/catkin_ws/build /home/cody/catkin_ws/build/CMakeFiles/move_quad_s.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/move_quad_s.dir/depend

