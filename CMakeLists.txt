cmake_minimum_required(VERSION 2.8.8)
set(PROJECT_NAME_STR panovis)
project(${PROJECT_NAME_STR} C CXX)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-Wall -ansi -Wno-deprecated -pthread)
endif()


set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)
set(EXT_PROJECTS_DIR ${PROJECT_SOURCE_DIR}/ext)
include_directories(${COMMON_INCLUDES})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


file(GLOB LIB_SRC_FILES ${PROJECT_SOURCE_DIR}/lib/*.cpp)
add_library( rvision ${LIB_SRC_FILES})

#-------------------
# Generate Camera Predict Data
#-------------------

add_executable(generate_camera_predict  ${PROJECT_SOURCE_DIR}/tests/generate_camera_predict.cpp)
target_link_libraries(generate_camera_predict ${CERES_LIBRARIES} rvision)

#-------------------
# Test
#-------------------
add_subdirectory(${EXT_PROJECTS_DIR}/gtest-1.7.0)

enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})


set(PROJECT_TEST_NAME ${PROJECT_NAME_STR}_test)

file(GLOB TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/tests/*_test.cpp)
add_executable(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})
target_link_libraries(${PROJECT_TEST_NAME}  gtest gtest_main rvision ${CERES_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(${PROJECT_TEST_NAME} googletest)


add_test(test1 ${PROJECT_TEST_NAME})