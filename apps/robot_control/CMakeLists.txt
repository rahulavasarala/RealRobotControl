set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MUJOCO_SAI_CONTROL_BINARY_DIR}/robot_control)
add_executable(robot_control controller.cpp)
target_link_libraries(robot_control ${MUJOCO_SAI_COMMON_LIBRARIES})