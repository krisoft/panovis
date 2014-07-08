cmake_minimum_required(VERSION 2.8.8)
set(PROJECT_NAME_STR panovis)
project(${PROJECT_NAME_STR} C CXX)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-Wall -ansi -Wno-deprecated -pthread)
endif()


set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)
set(EXT_PROJECTS_DIR ${PROJECT_SOURCE_DIR}/ext)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(${COMMON_INCLUDES})

file(GLOB LIB_SRC_FILES ${PROJECT_SOURCE_DIR}/lib/*.cpp)
add_library( rvision ${LIB_SRC_FILES})



#-------------------
# Test
#-------------------
add_subdirectory(${EXT_PROJECTS_DIR}/gtest-1.7.0)

enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})


set(PROJECT_TEST_NAME ${PROJECT_NAME_STR}_test)

file(GLOB TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/tests/*.cpp)
add_executable(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})
target_link_libraries(${PROJECT_TEST_NAME}  gtest gtest_main rvision ${CERES_LIBRARIES})
add_dependencies(${PROJECT_TEST_NAME} googletest)


add_test(test1 ${PROJECT_TEST_NAME})