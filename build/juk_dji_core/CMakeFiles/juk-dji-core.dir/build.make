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
CMAKE_SOURCE_DIR = /home/ubuntu/juk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/juk/build

# Include any dependencies generated for this target.
include juk_dji_core/CMakeFiles/juk-dji-core.dir/depend.make

# Include the progress variables for this target.
include juk_dji_core/CMakeFiles/juk-dji-core.dir/progress.make

# Include the compile flags for this target's objects.
include juk_dji_core/CMakeFiles/juk-dji-core.dir/flags.make

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o: juk_dji_core/CMakeFiles/juk-dji-core.dir/flags.make
juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o: ../juk_dji_core/common/dji_linux_environment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/juk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o -c /home/ubuntu/juk/juk_dji_core/common/dji_linux_environment.cpp

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.i"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/juk/juk_dji_core/common/dji_linux_environment.cpp > CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.i

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.s"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/juk/juk_dji_core/common/dji_linux_environment.cpp -o CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.s

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.requires:

.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.requires

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.provides: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.requires
	$(MAKE) -f juk_dji_core/CMakeFiles/juk-dji-core.dir/build.make juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.provides.build
.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.provides

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.provides.build: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o


juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o: juk_dji_core/CMakeFiles/juk-dji-core.dir/flags.make
juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o: ../juk_dji_core/common/dji_linux_helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/juk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o -c /home/ubuntu/juk/juk_dji_core/common/dji_linux_helpers.cpp

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.i"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/juk/juk_dji_core/common/dji_linux_helpers.cpp > CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.i

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.s"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/juk/juk_dji_core/common/dji_linux_helpers.cpp -o CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.s

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.requires:

.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.requires

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.provides: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.requires
	$(MAKE) -f juk_dji_core/CMakeFiles/juk-dji-core.dir/build.make juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.provides.build
.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.provides

juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.provides.build: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o


juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o: juk_dji_core/CMakeFiles/juk-dji-core.dir/flags.make
juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o: ../juk_dji_core/juk-dji-core.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/juk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o -c /home/ubuntu/juk/juk_dji_core/juk-dji-core.cpp

juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.i"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/juk/juk_dji_core/juk-dji-core.cpp > CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.i

juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.s"
	cd /home/ubuntu/juk/build/juk_dji_core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/juk/juk_dji_core/juk-dji-core.cpp -o CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.s

juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.requires:

.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.requires

juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.provides: juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.requires
	$(MAKE) -f juk_dji_core/CMakeFiles/juk-dji-core.dir/build.make juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.provides.build
.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.provides

juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.provides.build: juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o


# Object files for target juk-dji-core
juk__dji__core_OBJECTS = \
"CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o" \
"CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o" \
"CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o"

# External object files for target juk-dji-core
juk__dji__core_EXTERNAL_OBJECTS =

bin/juk-dji-core: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o
bin/juk-dji-core: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o
bin/juk-dji-core: juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o
bin/juk-dji-core: juk_dji_core/CMakeFiles/juk-dji-core.dir/build.make
bin/juk-dji-core: /opt/ros/melodic/lib/libroscpp.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_signals.so
bin/juk-dji-core: /opt/ros/melodic/lib/librosconsole.so
bin/juk-dji-core: /opt/ros/melodic/lib/librosconsole_log4cxx.so
bin/juk-dji-core: /opt/ros/melodic/lib/librosconsole_backend_interface.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_regex.so
bin/juk-dji-core: /opt/ros/melodic/lib/libroscpp_serialization.so
bin/juk-dji-core: /opt/ros/melodic/lib/libxmlrpcpp.so
bin/juk-dji-core: /opt/ros/melodic/lib/librostime.so
bin/juk-dji-core: /opt/ros/melodic/lib/libcpp_common.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_system.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_thread.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libpthread.so
bin/juk-dji-core: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
bin/juk-dji-core: /usr/local/lib/libdjiosdk-core.a
bin/juk-dji-core: /usr/local/lib/libLinuxChrono_lib.a
bin/juk-dji-core: /usr/local/lib/libGeoMath_lib.a
bin/juk-dji-core: juk_dji_core/CMakeFiles/juk-dji-core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/juk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../bin/juk-dji-core"
	cd /home/ubuntu/juk/build/juk_dji_core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/juk-dji-core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
juk_dji_core/CMakeFiles/juk-dji-core.dir/build: bin/juk-dji-core

.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/build

juk_dji_core/CMakeFiles/juk-dji-core.dir/requires: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_environment.cpp.o.requires
juk_dji_core/CMakeFiles/juk-dji-core.dir/requires: juk_dji_core/CMakeFiles/juk-dji-core.dir/common/dji_linux_helpers.cpp.o.requires
juk_dji_core/CMakeFiles/juk-dji-core.dir/requires: juk_dji_core/CMakeFiles/juk-dji-core.dir/juk-dji-core.cpp.o.requires

.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/requires

juk_dji_core/CMakeFiles/juk-dji-core.dir/clean:
	cd /home/ubuntu/juk/build/juk_dji_core && $(CMAKE_COMMAND) -P CMakeFiles/juk-dji-core.dir/cmake_clean.cmake
.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/clean

juk_dji_core/CMakeFiles/juk-dji-core.dir/depend:
	cd /home/ubuntu/juk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/juk /home/ubuntu/juk/juk_dji_core /home/ubuntu/juk/build /home/ubuntu/juk/build/juk_dji_core /home/ubuntu/juk/build/juk_dji_core/CMakeFiles/juk-dji-core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : juk_dji_core/CMakeFiles/juk-dji-core.dir/depend

