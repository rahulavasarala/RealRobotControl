/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include <iostream>
#include <string>
#include "redis_keys.h"

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

int main(int argc, char** argv) {
	// Location of URDF files specifying world and robot information
	static const string robot_file = std::string(URDF_PATH) + "/panda_arm_gripper.urdf";
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);

	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	auto dof = robot->dof();

	VectorXd control_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.17);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto motion_force_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	motion_force_task->setPosControlGains(400, 40, 0);
	motion_force_task->setOriControlGains(400, 40, 0);

	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	runloop = true;

	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);
	Vector3d offset = Vector3d(0, 0.3, 0.3);

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

	
		// update task model
		N_prec.setIdentity();
		motion_force_task->setGoalPosition(offset);
		motion_force_task->setGoalOrientation(Matrix3d::Identity());
		motion_force_task->updateTaskModel(N_prec);
		joint_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());

		control_torques = motion_force_task->computeTorques();
		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);

	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * control_torques);

	return 0;
}