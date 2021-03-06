CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT(rgbd_slam_basic)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(G2O REQUIRED)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include
                     ${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


add_library(${PROJECT_NAME} SHARED
main.cpp

src/transform.cpp
src/slam.cpp
src/frame.cpp

include/transform.h
include/slam.h
include/frame.h
)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src)

add_executable(main main.cpp)
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} 
${OpenCV_LIBS}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
target_link_libraries(main ${PROJECT_NAME})







