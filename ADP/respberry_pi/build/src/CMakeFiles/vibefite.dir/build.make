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
CMAKE_SOURCE_DIR = /home/ubuntu/project/opencvpro/vibe

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/project/opencvpro/vibe/build

# Include any dependencies generated for this target.
include src/CMakeFiles/vibefite.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/vibefite.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/vibefite.dir/flags.make

src/CMakeFiles/vibefite.dir/main.cpp.o: src/CMakeFiles/vibefite.dir/flags.make
src/CMakeFiles/vibefite.dir/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/project/opencvpro/vibe/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/vibefite.dir/main.cpp.o"
	cd /home/ubuntu/project/opencvpro/vibe/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vibefite.dir/main.cpp.o -c /home/ubuntu/project/opencvpro/vibe/src/main.cpp

src/CMakeFiles/vibefite.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vibefite.dir/main.cpp.i"
	cd /home/ubuntu/project/opencvpro/vibe/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/project/opencvpro/vibe/src/main.cpp > CMakeFiles/vibefite.dir/main.cpp.i

src/CMakeFiles/vibefite.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vibefite.dir/main.cpp.s"
	cd /home/ubuntu/project/opencvpro/vibe/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/project/opencvpro/vibe/src/main.cpp -o CMakeFiles/vibefite.dir/main.cpp.s

src/CMakeFiles/vibefite.dir/main.cpp.o.requires:

.PHONY : src/CMakeFiles/vibefite.dir/main.cpp.o.requires

src/CMakeFiles/vibefite.dir/main.cpp.o.provides: src/CMakeFiles/vibefite.dir/main.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/vibefite.dir/build.make src/CMakeFiles/vibefite.dir/main.cpp.o.provides.build
.PHONY : src/CMakeFiles/vibefite.dir/main.cpp.o.provides

src/CMakeFiles/vibefite.dir/main.cpp.o.provides.build: src/CMakeFiles/vibefite.dir/main.cpp.o


# Object files for target vibefite
vibefite_OBJECTS = \
"CMakeFiles/vibefite.dir/main.cpp.o"

# External object files for target vibefite
vibefite_EXTERNAL_OBJECTS =

bin/vibefite: src/CMakeFiles/vibefite.dir/main.cpp.o
bin/vibefite: src/CMakeFiles/vibefite.dir/build.make
bin/vibefite: lib/liblibvibe.a
bin/vibefite: /usr/local/lib/libopencv_videostab.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_ts.a
bin/vibefite: /usr/local/lib/libopencv_superres.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_stitching.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_contrib.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_nonfree.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_ocl.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_gpu.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_photo.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_objdetect.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_legacy.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_video.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_ml.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_calib3d.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_features2d.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_highgui.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_imgproc.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_flann.so.2.4.9
bin/vibefite: /usr/local/lib/libopencv_core.so.2.4.9
bin/vibefite: src/CMakeFiles/vibefite.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/project/opencvpro/vibe/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/vibefite"
	cd /home/ubuntu/project/opencvpro/vibe/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vibefite.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/vibefite.dir/build: bin/vibefite

.PHONY : src/CMakeFiles/vibefite.dir/build

src/CMakeFiles/vibefite.dir/requires: src/CMakeFiles/vibefite.dir/main.cpp.o.requires

.PHONY : src/CMakeFiles/vibefite.dir/requires

src/CMakeFiles/vibefite.dir/clean:
	cd /home/ubuntu/project/opencvpro/vibe/build/src && $(CMAKE_COMMAND) -P CMakeFiles/vibefite.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/vibefite.dir/clean

src/CMakeFiles/vibefite.dir/depend:
	cd /home/ubuntu/project/opencvpro/vibe/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/project/opencvpro/vibe /home/ubuntu/project/opencvpro/vibe/src /home/ubuntu/project/opencvpro/vibe/build /home/ubuntu/project/opencvpro/vibe/build/src /home/ubuntu/project/opencvpro/vibe/build/src/CMakeFiles/vibefite.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/vibefite.dir/depend

