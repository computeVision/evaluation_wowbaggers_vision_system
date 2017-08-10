cmake_minimum_required(VERSION 2.8)
project(tedusar_detect_evaluation)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp std_msgs cv_bridge genmsg image_transport sensor_msgs message_generation)
find_package(OpenCV)
find_package(TinyXML)
find_package(rostest)
# find_package(rostest REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

add_message_files(DIRECTORY msg
   FILES RectImage.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)
catkin_package()

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

add_executable(image_publisher src/image_publisher.cpp src/abstract_choice.cpp src/my_exception.cpp)
add_executable(overlapping_rect src/abstract_choice.cpp src/my_exception.cpp src/overlapping_rect.cpp)
add_executable(roc_choice src/roc_choice.cpp src/abstract_choice.cpp src/my_exception.cpp)


#add_executable(detector src/detector_choice.cpp src/abstract_choice.cpp src/my_exception.cpp)

target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_INCLUDE_DIRS} ${TinyXML_LIBRARIES})
target_link_libraries(overlapping_rect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_INCLUDE_DIRS} ${TinyXML_LIBRARIES})
target_link_libraries(roc_choice ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_INCLUDE_DIRS} ${TinyXML_LIBRARIES})

#target_link_libraries(detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_INCLUDE_DIRS} ${TinyXML_LIBRARIES})

add_dependencies(roc_choice ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(overlapping_rect ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(image_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_rostest_gtest(utest test/utest.cpp)
#add_library(project_lib src/abstract_choice.cpp src/my_exception.cpp src/overlapping_rect.cpp)
#catkin_add_gtest(utest test/utest.cpp)
#target_link_libraries(utest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_INCLUDE_DIRS})

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tedusar_detect_evaluation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)