# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tedusar_detect_evaluation: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itedusar_detect_evaluation:/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tedusar_detect_evaluation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg" NAME_WE)
add_custom_target(_tedusar_detect_evaluation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tedusar_detect_evaluation" "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg" "std_msgs/Header:sensor_msgs/Image"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tedusar_detect_evaluation
  "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tedusar_detect_evaluation
)

### Generating Services

### Generating Module File
_generate_module_cpp(tedusar_detect_evaluation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tedusar_detect_evaluation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tedusar_detect_evaluation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tedusar_detect_evaluation_generate_messages tedusar_detect_evaluation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg" NAME_WE)
add_dependencies(tedusar_detect_evaluation_generate_messages_cpp _tedusar_detect_evaluation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tedusar_detect_evaluation_gencpp)
add_dependencies(tedusar_detect_evaluation_gencpp tedusar_detect_evaluation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tedusar_detect_evaluation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tedusar_detect_evaluation
  "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tedusar_detect_evaluation
)

### Generating Services

### Generating Module File
_generate_module_lisp(tedusar_detect_evaluation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tedusar_detect_evaluation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tedusar_detect_evaluation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tedusar_detect_evaluation_generate_messages tedusar_detect_evaluation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg" NAME_WE)
add_dependencies(tedusar_detect_evaluation_generate_messages_lisp _tedusar_detect_evaluation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tedusar_detect_evaluation_genlisp)
add_dependencies(tedusar_detect_evaluation_genlisp tedusar_detect_evaluation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tedusar_detect_evaluation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tedusar_detect_evaluation
  "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tedusar_detect_evaluation
)

### Generating Services

### Generating Module File
_generate_module_py(tedusar_detect_evaluation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tedusar_detect_evaluation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tedusar_detect_evaluation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tedusar_detect_evaluation_generate_messages tedusar_detect_evaluation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/plorenz/ros_pkg/src/tedusar_perception/tedusar_detect_evaluation/msg/RectImage.msg" NAME_WE)
add_dependencies(tedusar_detect_evaluation_generate_messages_py _tedusar_detect_evaluation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tedusar_detect_evaluation_genpy)
add_dependencies(tedusar_detect_evaluation_genpy tedusar_detect_evaluation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tedusar_detect_evaluation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tedusar_detect_evaluation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tedusar_detect_evaluation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(tedusar_detect_evaluation_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(tedusar_detect_evaluation_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tedusar_detect_evaluation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tedusar_detect_evaluation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(tedusar_detect_evaluation_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(tedusar_detect_evaluation_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tedusar_detect_evaluation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tedusar_detect_evaluation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tedusar_detect_evaluation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(tedusar_detect_evaluation_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(tedusar_detect_evaluation_generate_messages_py std_msgs_generate_messages_py)
