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

void updateRobotState(std::shared_ptr<SaiModel::SaiModel> robot, SaiCommon::RedisClient* redis_client) {
   VectorXd robot_q = redis_client->getEigen(JOINT_ANGLES_KEY);

   if (robot_q.size() == 7) {
       robot_q.conservativeResize(9);
       robot_q.tail(2).setZero();
   }

   VectorXd robot_dq = redis_client->getEigen(JOINT_VELOCITIES_KEY);


   if (robot_dq.size() == 7) {
       robot_dq.conservativeResize(9);
       robot_dq.tail(2).setZero();
   }


   robot->setQ(robot_q);
   robot->setDq(robot_dq);
   robot->updateModel();

}


int main(int argc, char** argv) {
	// Location of URDF files specifying world and robot information
	static const string robot_file = std::string(URDF_PATH) + "/panda_arm_gripper.urdf";

	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);

	updateRobotState(robot, &redis_client);

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
	Vector3d offset = Vector3d(0, 0, 0.3);

	Matrix3d custom_orient;
	custom_orient << 1,0,0,
					0,-1,0,
					0,0,-1;

	int loop_count = 0;

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		updateRobotState(robot, &redis_client);

		// update task model
		N_prec.setIdentity();
		motion_force_task->updateTaskModel(N_prec);

		motion_force_task->setGoalPosition(offset);
		motion_force_task->setGoalOrientation(custom_orient);

		control_torques = motion_force_task->computeTorques();
		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);

		if (loop_count % 1000 == 0) {
			std::cout << "control torques: " << control_torques << std::endl;
			// std::cout << "joint angles: " << redis_client.getEigen(JOINT_ANGLES_KEY) << std::endl;
			// std::cout << "joint vels: " << redis_client.getEigen(JOINT_VELOCITIES_KEY) << std::endl;
		}

		loop_count += 1;

	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * control_torques);

	return 0;
}