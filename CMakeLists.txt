cmake_minimum_required(VERSION 3.5)

PROJECT(HandEyeCat)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})

IF(MSVC)
	set(WXN PRIVATE /W0 /WX)

ELSEIF(UNIX)
	set(WXN PRIVATE -W -Wall -Wno-unused 
			-Wno-strict-aliasing -Wno-format-overflow  -Wno-int-in-bool-context 
			-Wno-sign-compare -Wno-reorder -Wextra -Wno-variadic-macros -Wno-format) 
ENDIF()

# Eigen Related
if(WIN32)
	set(EIGEN3_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen-src)
	include_directories(${EIGEN3_INCLUDE_DIRS})
	include_directories(${EIGEN3_INCLUDE_DIRS}/Eigen)
else(WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")
	find_package(Eigen3 REQUIRED)
	include_directories(${EIGEN3_INCLUDE_DIRS})
endif(WIN32)


add_executable(${PROJECT_NAME}
	src/main.cpp
	src/common.cpp)

target_compile_options(${PROJECT_NAME} ${WXN}) 