# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gps_driver: 2 messages, 0 services")

set(MSG_I_FLAGS "-Igps_driver:/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gps_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" NAME_WE)
add_custom_target(_gps_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gps_driver" "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" NAME_WE)
add_custom_target(_gps_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gps_driver" "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_driver
)
_generate_msg_cpp(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(gps_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gps_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gps_driver_generate_messages gps_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_cpp _gps_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_cpp _gps_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_driver_gencpp)
add_dependencies(gps_driver_gencpp gps_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_driver
)
_generate_msg_eus(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(gps_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gps_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gps_driver_generate_messages gps_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_eus _gps_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_eus _gps_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_driver_geneus)
add_dependencies(gps_driver_geneus gps_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_driver
)
_generate_msg_lisp(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(gps_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gps_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gps_driver_generate_messages gps_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_lisp _gps_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_lisp _gps_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_driver_genlisp)
add_dependencies(gps_driver_genlisp gps_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_driver
)
_generate_msg_nodejs(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gps_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gps_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gps_driver_generate_messages gps_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_nodejs _gps_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_nodejs _gps_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_driver_gennodejs)
add_dependencies(gps_driver_gennodejs gps_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_driver
)
_generate_msg_py(gps_driver
  "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_driver
)

### Generating Services

### Generating Module File
_generate_module_py(gps_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gps_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gps_driver_generate_messages gps_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customgps.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_py _gps_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sherly/Desktop/code/EECE5554/integration_ws/src/gps_driver/msg/Customrtk.msg" NAME_WE)
add_dependencies(gps_driver_generate_messages_py _gps_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_driver_genpy)
add_dependencies(gps_driver_genpy gps_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gps_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gps_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gps_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gps_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gps_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
