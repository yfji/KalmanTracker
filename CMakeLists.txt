project(KalmanTracker)
cmake_minimum_required(VERSION 3.0)

set(OpenCV_DIR /home/yfji/packages/opencv-3.4.0/build-cpu/install/share/OpenCV)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11 -fopenmp -fPIC")
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
	${PROJECT_SOURCE_DIR}/src
	/usr/include
	)

link_directories(
	${OpenCV_LIB_DIRS}
	/usr/lib
	/usr/local/lib
	/usr/lib/x86_64-linux-gnu
	)

add_executable(KalmanTracker ${DIR_SRCS})
target_link_libraries(KalmanTracker ${OpenCV_LIBS} pthread)
	
