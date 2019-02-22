cmake_minimum_required( VERSION 2.8 )
project( mapping_project )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

#opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#PCL
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#eigen
include_directories( "/usr/include/eigen3" )

#g2o
include_directories(/use/local/include)
link_directories(/usr/local/lib)

#cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})


#csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

#project include
include_directories( ${PROJECT_SOURCE_DIR}/include )

#source files
set(SOURCE_FILES 
	src/main.cpp
	src/converter.cpp
	src/depth_filter.cpp
	src/frame.cpp
	src/mapping.cpp
	src/optimizer.cpp
	src/pcl.cpp
	src/seed.cpp
	src/utils.cpp
)

add_executable(${PROJECT_NAME}  ${SOURCE_FILES})

SET(G2O_LIBS g2o_core g2o_csparse_extension g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_stuff g2o_types_sba g2o_types_icp g2o_types_sim3)

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBS} ${CHOLMOD_LIBRARIES} ${CSPARSE_LIBRARIES} )


