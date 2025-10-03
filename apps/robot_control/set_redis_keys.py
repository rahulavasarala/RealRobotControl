import redis
import json
import numpy as np

JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_velocities"
JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_positions"
JOINT_TORQUES_COMMANDED_KEY = "sai::sensors::FrankaRobot::joint_torques"
FORCE_SENSOR_KEY = "sai::sensors::FrankaRobot::ft_sensor::force"
MOMENT_SENSOR_KEY = "sai::sensors::FrankaRobot::ft_sensor::torque"

redis_client = redis.Redis()
joint_angles = np.array([0.007, -0.335, -0.037, -2.62894, -0.02888, 2.4217, 0.758973, 0, 0])
joint_velocities = np.zeros(9)
joint_torques = np.zeros(9)

redis_client.set(JOINT_ANGLES_KEY, json.dumps(joint_angles.tolist()))
redis_client.set(JOINT_VELOCITIES_KEY, json.dumps(joint_velocities.tolist()))
redis_client.set(JOINT_TORQUES_COMMANDED_KEY, json.dumps(joint_torques.tolist()))

