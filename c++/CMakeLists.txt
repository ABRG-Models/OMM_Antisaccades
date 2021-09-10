# AUTOMOC requires 2.8.6. Per-target include directories would require 2.8.8
cmake_minimum_required(VERSION 2.8.6)
PROJECT(worldDataMaker)

### Build subdirs. Also add them to include path.
#add_subdirectory(Scene3D)
#include_directories(Scene3D)

### Header file
file(GLOB SOURCE_HEADERS *.h)

### Add a per-user include directory in case that's useful
include_directories($ENV{HOME}/usr/include)
link_directories($ENV{HOME}/usr/lib)

### Executable (there is no exe for this proj; only the Brahms component)
set(TARGET worldtest CACHE TYPE STRING)
set(SOURCE ${SOURCE_HEADERS} worldtest.cpp worldframe.cpp eyeframe.cpp)
add_executable(${TARGET} ${SOURCE})
set_target_properties (${TARGET} PROPERTIES COMPILE_FLAGS "-g")
target_link_libraries(${TARGET} jsoncpp)

set(TARGET_WTCO worldtest_coords CACHE TYPE STRING)
set(SOURCE_WTCO ${SOURCE_HEADERS} worldtest_coords.cpp worldframe.cpp eyeframe.cpp)
add_executable(${TARGET_WTCO} ${SOURCE_WTCO})
set_target_properties (${TARGET_WTCO} PROPERTIES COMPILE_FLAGS "-g")
target_link_libraries(${TARGET_WTCO} jsoncpp)

set(TARGET_WTSL worldtest_singlelum CACHE TYPE STRING)
add_executable(${TARGET_WTSL} ${SOURCE_HEADERS} worldtest_singlelum.cpp worldframe.cpp eyeframe.cpp)
set_target_properties (${TARGET_WTSL} PROPERTIES COMPILE_FLAGS "-g")
target_link_libraries(${TARGET_WTSL} jsoncpp)

set(WM_TARGET weightmap CACHE TYPE STRING)
set(WM_SOURCE ${SOURCE_HEADERS} weightmap.cpp worldframe.cpp eyeframe.cpp)
add_executable(${WM_TARGET} ${WM_SOURCE})
set_target_properties (${WM_TARGET} PROPERTIES COMPILE_FLAGS "-g")

set(DH_TARGET doublehump CACHE TYPE STRING)
set(DH_SOURCE ${SOURCE_HEADERS} doublehump.cpp worldframe.cpp eyeframe.cpp)
add_executable(${DH_TARGET} ${DH_SOURCE})
set_target_properties (${DH_TARGET} PROPERTIES COMPILE_FLAGS "-g")

set(WS_TARGET worldseries CACHE TYPE STRING)
set(WS_SOURCE ${SOURCE_HEADERS} worldseries.cpp worldframe.cpp eyeframe.cpp)
add_executable(${WS_TARGET} ${WS_SOURCE})
set_target_properties (${WS_TARGET} PROPERTIES COMPILE_FLAGS "-g")

set(LRUD_TARGET leftrightupdown CACHE TYPE STRING)
set(LRUD_SOURCE ${SOURCE_HEADERS} leftrightupdown.cpp worldframe.cpp eyeframe.cpp)
add_executable(${LRUD_TARGET} ${LRUD_SOURCE})
set_target_properties (${LRUD_TARGET} PROPERTIES COMPILE_FLAGS "-g")

### Set the SYSTEMML_INSTALL_PATH once only
set(SYSTEMML_INSTALL_PATH $ENV{SYSTEMML_INSTALL_PATH} CACHE PATH
    "Installation location of Brahms (e.g. ~/SystemML)")

### Include directories for Brahms components.
include_directories(/usr/include/brahms)
include_directories(/var/lib/brahms/Namespace)
include_directories(${SYSTEMML_INSTALL_PATH}/BRAHMS/include)
include_directories(${SYSTEMML_INSTALL_PATH}/Namespace)
include_directories(/usr/include/eigen3)
include_directories($ENV{HOME}/usr/include)

### Link dirs for Brahms components
link_directories(${SYSTEMML_INSTALL_PATH}/BRAHMS/bin)

### Brahms Component
set(BC_TARGET worldDataMaker CACHE TYPE STRING)
set(BC_SOURCE ${SOURCE_HEADERS} worldDataMaker.cpp worldframe.cpp eyeframe.cpp)
add_library(${BC_TARGET} SHARED ${BC_SOURCE})
# "" rather than "lib" to give Brahms style "component.so" not "libcomponent.so":
set_target_properties(${BC_TARGET} PROPERTIES PREFIX "")
set_target_properties (${BC_TARGET} PROPERTIES COMPILE_DEFINITIONS "__GLN__")
set_target_properties (${BC_TARGET} PROPERTIES COMPILE_FLAGS "-fPIC -Werror -pthread -msse2 -O3")
# Require jsoncpp linking for worldDataMaker
find_package(jsoncpp REQUIRED)
get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${JSON_INC_PATH})
target_link_libraries(${BC_TARGET} jsoncpp_lib)
##########################################################################################

### Brahms Component for single centroid of whole map
set(CENTROID centroid CACHE TYPE STRING)
set(CENTROID_SOURCE ${SOURCE_HEADERS} centroid.cpp)
add_library(${CENTROID} SHARED ${CENTROID_SOURCE})
# "" rather than "lib" to give Brahms style "centroid.so" not "libcentroid.so":
set_target_properties(${CENTROID} PROPERTIES PREFIX "")
set_target_properties (${CENTROID} PROPERTIES COMPILE_DEFINITIONS "__GLN__")
set_target_properties (${CENTROID} PROPERTIES COMPILE_FLAGS "-fPIC -Werror -pthread -O3")
##########################################################################################

### Brahms Component for pseudoeye
set(PSEUDOEYE pseudoeye)
set(PSEUDOEYE_SOURCE ${SOURCE_HEADERS} pseudoeye.cpp)
add_library(${PSEUDOEYE} SHARED ${PSEUDOEYE_SOURCE})
set_target_properties(${PSEUDOEYE} PROPERTIES PREFIX "")
set_target_properties (${PSEUDOEYE} PROPERTIES COMPILE_DEFINITIONS "__GLN__")
set_target_properties (${PSEUDOEYE} PROPERTIES COMPILE_FLAGS "-fPIC -Werror -pthread -msse2 -O3")
##########################################################################################
