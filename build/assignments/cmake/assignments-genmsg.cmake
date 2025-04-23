# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "assignments: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/home/asmar/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(assignments_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" NAME_WE)
add_custom_target(_assignments_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assignments" "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" ""
)

get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" NAME_WE)
add_custom_target(_assignments_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assignments" "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignments
)
_generate_srv_cpp(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignments
)

### Generating Module File
_generate_module_cpp(assignments
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignments
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(assignments_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(assignments_generate_messages assignments_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" NAME_WE)
add_dependencies(assignments_generate_messages_cpp _assignments_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" NAME_WE)
add_dependencies(assignments_generate_messages_cpp _assignments_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignments_gencpp)
add_dependencies(assignments_gencpp assignments_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignments_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignments
)
_generate_srv_eus(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignments
)

### Generating Module File
_generate_module_eus(assignments
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignments
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(assignments_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(assignments_generate_messages assignments_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" NAME_WE)
add_dependencies(assignments_generate_messages_eus _assignments_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" NAME_WE)
add_dependencies(assignments_generate_messages_eus _assignments_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignments_geneus)
add_dependencies(assignments_geneus assignments_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignments_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignments
)
_generate_srv_lisp(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignments
)

### Generating Module File
_generate_module_lisp(assignments
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignments
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(assignments_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(assignments_generate_messages assignments_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" NAME_WE)
add_dependencies(assignments_generate_messages_lisp _assignments_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" NAME_WE)
add_dependencies(assignments_generate_messages_lisp _assignments_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignments_genlisp)
add_dependencies(assignments_genlisp assignments_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignments_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignments
)
_generate_srv_nodejs(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignments
)

### Generating Module File
_generate_module_nodejs(assignments
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignments
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(assignments_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(assignments_generate_messages assignments_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" NAME_WE)
add_dependencies(assignments_generate_messages_nodejs _assignments_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" NAME_WE)
add_dependencies(assignments_generate_messages_nodejs _assignments_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignments_gennodejs)
add_dependencies(assignments_gennodejs assignments_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignments_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignments
)
_generate_srv_py(assignments
  "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignments
)

### Generating Module File
_generate_module_py(assignments
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignments
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(assignments_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(assignments_generate_messages assignments_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/CheckJointState.srv" NAME_WE)
add_dependencies(assignments_generate_messages_py _assignments_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/asmar/UNIGE/COGAR/COGAR_Assignment/src/assignments/srv/Speaker.srv" NAME_WE)
add_dependencies(assignments_generate_messages_py _assignments_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignments_genpy)
add_dependencies(assignments_genpy assignments_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignments_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignments)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignments
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(assignments_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignments)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignments
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(assignments_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignments)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignments
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(assignments_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignments)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignments
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(assignments_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignments)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignments\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignments
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(assignments_generate_messages_py std_msgs_generate_messages_py)
endif()
