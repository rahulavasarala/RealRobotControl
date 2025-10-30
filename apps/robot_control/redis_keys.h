#include <string.h>

const std::string JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_positions";
const std::string JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_velocities";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::commands::FrankaRobot::control_torques";

const std::string FORCE_SENSOR_KEY = "sai::sensors::FrankaRobot::ft_sensor::end-effector::force";
const std::string MOMENT_SENSOR_KEY = "sai::sensors::FrankaRobot::ft_sensor::end-effector::moment";
const std::string FORCE_TORQUE_KEY = "sai::sensors::FrankaRobot::ft_sensor::end-effector::force_torque";

const std::string SMOOTHED_FORCE = "sai::sensors::FrankaRobot::ft_sensor::smoothed_force";
const std::string SMOOTHED_TORQUE = "sai::sensors::FrankaRobot::ft_sensor::smoothed_torque";


//There will be keys that the python environment will come up with, which will be directed for the mft controller

const std::string MOTION_FORCE_AXIS = "sai::sensors::FrankaRobot::motion_force_axis"; //Type Eigen VectorXD x , y, z, dim
const std::string DESIRED_FORCE = "sai::sensors::FrankaRobot::desired_force";//This is the desired force we want to exert

const std::string TARGET_POS = "sai::sensors::FrankaRobot::target_pos";
const std::string TARGET_ORIENT = "sai::sensors::FrankaRobot::target_orient";

const std::string CONTROL_POINT_ORIENT = "sai::sensors::FrankaRobot::control_point_orient";
const std::string CONTROL_POINT_POS = "sai::sensors::FrankaRobot::control_point_pos";
const std::string CONTROL_POINT_VEL = "sai::sensors::FrankaRobot::control_point_vel";
const std::string MASS_MATRIX = "sai::sensors::FrankaRobot::mass_matrix";


