/** 
 * @file 		bias_esimate.cpp
 * @brief 		Controller file to determine the force sensor bias and load mass.
 * 				The force sensor bias is the average force/moment reading over
 * 				4 configurations, since the net force/moment reading should read 0.
 * 				The sensor can be loaded with a mass since the configurations 
 * 				are chosen such that sum of the measurements should be 0 (symmetric orientations).
 * 				The final nominal panda posture gives the estimated load mass from the force bias.
 * 
 */

#include "SaiModel.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "SaiPrimitives.h"

#include <fstream>
#include <tinyxml2.h>

#include <iostream>
#include <string>
#include "redis_keys.h"
#include <cmath>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

const string panda_file = std::string(URDF_PATH) + "/panda_arm.urdf";
const string robot_name = "panda_arm";  // iiwa7 or iiwa14 or panda_arm
const string force_sensor_name = "ati";  // opto or ati
const string bias_fname = "/Users/rahulavasarala/Desktop/OpenSai/RealRobotControl/calibration/bias_measurement.xml";

string CONTROLLER_RUNNING_KEY = "sai::sensors::FrankaRobot::controller";

unsigned long long controller_counter = 0;
const bool inertia_regularization = true;

// state machine 
enum State {
	POSTURE = 0,	
	CALIBRATION
};

// simulation flag 
// const bool flag_simulation = true;
const bool flag_simulation = false;

void writeXml(const string& file_name, const double& mass, const Vector3d& com, const VectorXd& sensor_bias);

int sign(double value) {
    if (value > 0) {
        return 1;
    } else if (value < 0) {
        return -1;
    } else {
        return 0; // Value is zero
    }
}

int main() {

	Vector7d q0, q1, q2, q3, q4, q5;
	if (robot_name == "panda_arm") {
		q0 << 90, 0, 0, -90, 0, 180, -90;  // + y
		q1 << 90, 0, 0, -90, 0, 180, 0;  // + x
		q2 << 90, 0, 0, -90, 0, 180, 90;  // - y
		q3 << 90, 0, 0, -90, 90, 180, 90;  // - x
		q4 << 90, 0, 0, -90, 0, 90, 0;  // - z 
	}

	std::vector<Vector7d> calib_config = { (M_PI / 180) * q0, (M_PI / 180) * q1, (M_PI / 180) * q2, (M_PI / 180) * q3, (M_PI / 180) * q4 };

	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	redis_client.set(CONTROLLER_RUNNING_KEY, "0");

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robot
	Affine3d T_robot = Affine3d::Identity();
	string robot_file = panda_file;
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// starting state 
	int state = POSTURE;

	// ***************** //
	// joint (posture) task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	VectorXd max_velocity(7);

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->setGains(400, 20, 20);

	// set the desired posture
	VectorXd q_desired = calib_config[0];
	joint_task->setGoalPosition(q_desired);

	const double QTOL = 5e-2;

	// setup bias measurement analysis
	std::vector<Vector6d> bias_measurement_vector;
	Vector6d bias_measurement = Vector6d::Zero();  // averaged bias measurement at the end
	Vector6d bias_sum = Vector6d::Zero();  // bias sum at one configuration 
	Vector6d force_moment_reading;
	double load_mass;
	int n_measurements = 5 * 1000;  // 5 seconds measurement
	int wait_count = 3 * 1000;  // 2 second wait before starting measurement after reaching goal configuration 
	int measurement_count = 0;  // current 
	int curr_count = 0;  // current loop counter to compare against measurements
	bool integrator_reset = false;

	// create a timer
	SaiCommon::LoopTimer timer(1000, 1e6);
	double start_time = timer.elapsedTime(); //secs
	int transient_count = 0;

	std::cout << "Made it to the while loop!" << std::endl;

	while (runloop) 
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));

		if (!flag_simulation) {
			MatrixXd M = redis_client.getEigen(MASS_MATRIX);
			
			if (inertia_regularization)	{
				M(4, 4) += 0.25;
				M(5, 5) += 0.25;
				M(6, 6) += 0.25;
			}
			robot->updateModel(M);
		} else {
			robot->updateModel();
		}

		force_moment_reading = redis_client.getEigen(FORCE_TORQUE_KEY);

		if (state == POSTURE) {
			// Update posture 
			joint_task->setGoalPosition(q_desired);

			// update task model
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			command_torques = joint_task->computeTorques();

			std::cout << "Position norm error: " << (robot->q() - q_desired).norm() << "\n";

			// add integrator if close enough
			if (joint_task->goalPositionReached(8e-1) && !integrator_reset) {
				std::cout << "Enabled integrator\n";
				joint_task->resetIntegrators();
				joint_task->setGains(400, 20, 200);
				integrator_reset = true;
			}

			// check exit conditions
			if (joint_task->goalPositionReached(QTOL)) {
				std::cout << "Starting calibration\n";
				cout << "Calibration Position: 0\n";
				state = CALIBRATION;  
				joint_task->setGoalPosition(q_desired);
				joint_task->setGains(400, 20, 200);
				continue;
			}
		} else if (state == CALIBRATION) {
			// set to desired calibration configuration
			joint_task->setGoalPosition(calib_config[measurement_count]);

			// update task model
			N_prec.setIdentity();			
			joint_task->updateTaskModel(N_prec);

			// compute torques
			command_torques = joint_task->computeTorques();

			if (joint_task->goalPositionReached(QTOL)) {

				curr_count++;

				if (curr_count > wait_count) {
					bias_sum += (force_moment_reading / n_measurements);
				}

				if (curr_count > n_measurements + wait_count) {
					// if last position, compute mass and moment bias for Mz
					if (measurement_count == calib_config.size() - 1) {
						// std::cout << "Measurement: " << ((bias_sum - bias_measurement) / n_measurements).transpose() << "\n";
						Matrix3d R = robot->rotation("link7");
						Vector3d sensor_gravity = R.transpose() * Vector3d(0, 0, -9.81);
						Vector3d force_measurement = bias_sum.head(3) - bias_measurement.head(3);
						load_mass = - force_measurement(2) / sensor_gravity(2);
						bias_measurement(5) = bias_sum(5);
						measurement_count++;
					} else {
						// moment bias for My for +y excitation and Mx for +x excitation
						if (measurement_count == 0) {  // +y excitation
							bias_measurement(4) = bias_sum(4);
						} else if (measurement_count == 1) {  // +x excitation
							bias_measurement(3) = bias_sum(3);
						}
						bias_measurement_vector.push_back(bias_sum);
						bias_measurement.head(3) += bias_sum.head(3) / 4;
						// cout << "Added measurement: " << ((1. / n_measurements) * bias_sum).transpose() << "\n";
						cout << "Calibration Position: " << measurement_count + 1 << "\n";
						measurement_count++;
						curr_count = 0;
						bias_sum.setZero();
					}

					if (measurement_count == calib_config.size()) {
						cout << "Finished Calibration" << endl;
						runloop = false; 
						redis_client.set(CONTROLLER_RUNNING_KEY, "0");
						redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);	
					}
				}
			}
		} 

		// send to redis
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// set controller running
		redis_client.set(CONTROLLER_RUNNING_KEY, "1");

		controller_counter++;
	}

	// Compute COM
	// First posture: +y up, thus mx, mz observed
	// Second posture: +x up, thus my, mz observed
	Vector3d y_excitation = ((bias_measurement_vector[2] - bias_measurement) / (load_mass * 9.81)).tail(3);
	double rz = y_excitation(0);
	double rx = - sign(rz) * y_excitation(2);  // opposite sign of rz for this measurement [mg*rz; 0; -mg*rx]
	Vector3d x_excitation = ((bias_measurement_vector[3] - bias_measurement) / (load_mass * 9.81)).tail(3);
	double ry = - sign(x_excitation(1)) * x_excitation(2);  // opposite sign of rz for this measurement [0; mg*rz; -mg*ry]
	Vector3d r_com = Vector3d(rx, ry, rz);

	// Verify
	Vector3d p_load = robot->rotation("link7").transpose() * Vector3d(0, 0, -9.81) * load_mass;
	Vector3d m_load = r_com.cross(p_load);
	VectorXd calibrated_force_moment = redis_client.getEigen(FORCE_TORQUE_KEY) - bias_measurement;
	calibrated_force_moment.head(3) += p_load;
	calibrated_force_moment.tail(3) += m_load;
	std::cout << "Calibrated force moment reading: " << calibrated_force_moment.transpose() << "\n";

	// Write to xml file
	std::cout << "Load mass: \n" << load_mass << "\n";
	std::cout << "COM: \n" << r_com.transpose() << "\n";
	writeXml(bias_fname, load_mass, r_com, bias_measurement);	

	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");

	return 0;
}

void writeXml(const string& file_name, const double& mass, const Vector3d& com, const VectorXd& sensor_bias) {
	if (sensor_bias.size() != 6)	{
		cout << "Bias should be a vector of length 6\nXml file not written" << endl;
		return;
	}

	cout << "Writing mass and bias to file " << file_name << endl;

	std::cout << "Going to write file to: " << file_name << std::endl;

	ofstream file;
	file.open(file_name);

	if (file.is_open()) {
		file << "<?xml version=\"1.0\" ?>\n";
		file << "<mass value=\"" << mass << "\"/>\n";
		file << "<com value=\"" << com.transpose() << "\"/>\n";
		file << "<force_bias value=\"" << sensor_bias.transpose() << "\"/>\n";
		file.close();
	} else {
		cout << "Could not create xml file" << endl;
	}
}