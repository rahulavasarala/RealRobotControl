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
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

//initialize all the vectors that talk to redis over here
bool sim = true;

Vector3d target_pos;
VectorXd target_orient;
VectorXd motion_force_axis;
Vector3d desired_force;
VectorXd robot_q;
VectorXd robot_dq;

VectorXd control_torques;
VectorXd sensed_force;
VectorXd sensed_torque;
Vector3d control_point_pos;
VectorXd control_point_orient;

//I need to have a period where I calibrate the force sensors for value tearing, which is neccessary for the force sensor
//values

class VectorBuffer {
public:
    VectorBuffer(int size, int dimension) 
        : max_size_(size), 
          current_sum_(VectorXd::Zero(dimension)), 
          dimension_(dimension) 
    {
        if (size <= 0 || dimension <= 0) {
            throw std::invalid_argument("Window size (L) and vector dimension (d) must be positive integers.");
        }
    }

    VectorXd addValue(const VectorXd& newValue) {
        if (newValue.size() != dimension_) {
            throw std::invalid_argument("Input vector dimension mismatch. Expected dimension " + std::to_string(dimension_) + ".");
        }

        current_sum_ += newValue;

        buffer_.push_back(newValue);

        if (buffer_.size() > max_size_) {
            const VectorXd& oldestValue = buffer_.front();
            current_sum_ -= oldestValue;

            buffer_.pop_front();
        }

        return getAverage();
    }

    VectorXd getAverage() const {
        if (buffer_.empty()) {
            return VectorXd::Zero(dimension_);
        }
        // Current average is the running sum divided by the number of elements (scalar division)
        return current_sum_ / (double)buffer_.size();
    }

    size_t getCurrentSize() const {
        return buffer_.size();
    }

private:
    std::deque<VectorXd> buffer_; 
    int max_size_;                 
    VectorXd current_sum_;       
    int dimension_;               
};

void init_keys( SaiCommon::RedisClient* redis_client) {
    //Set all the stuff to 
    target_pos = Vector3d(0.4, 0,0.4);
    target_orient.resize(4);
    target_orient << 1, 0,0,0;

    motion_force_axis.resize(4);
    motion_force_axis << 0,0,0,0;
    desired_force = Vector3d(0,0,0);

    control_point_pos.resize(3);
    control_point_pos << 0,0,0;

    control_point_orient.resize(4);
    control_point_orient << 0,1,0,0;

    sensed_force.resize(3);
    sensed_force << 0,0,0;
    sensed_torque.resize(3);
    sensed_torque << 0,0,0;

    if (sim) {
        robot_q = VectorXd::Zero(9);
        robot_dq = VectorXd::Zero(9);
    } else {
        robot_q = VectorXd::Zero(7);
        robot_dq = VectorXd::Zero(7);
    }
    control_torques.resize(9);
    control_torques = VectorXd::Zero(9);

    redis_client->setEigen(MOTION_FORCE_AXIS, motion_force_axis);
    redis_client->setEigen(DESIRED_FORCE, desired_force);
    redis_client->setEigen(TARGET_POS, target_pos);
    redis_client->setEigen(TARGET_ORIENT, target_orient);
    
}

void set_send_receive_keys(SaiCommon::RedisClient* redis_client) {
    redis_client->addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client->addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);
    redis_client->addToReceiveGroup(MOTION_FORCE_AXIS, motion_force_axis);
    redis_client->addToReceiveGroup(TARGET_POS, target_pos);
    redis_client->addToReceiveGroup(TARGET_ORIENT, target_orient);
    redis_client->addToReceiveGroup(DESIRED_FORCE, desired_force); 
    redis_client->addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client->addToReceiveGroup(FORCE_SENSOR_KEY, sensed_force);
    redis_client->addToReceiveGroup(MOMENT_SENSOR_KEY, sensed_torque);

}

void updateControlPointValues(std::shared_ptr<SaiPrimitives::MotionForceTask> motion_force_task, SaiCommon::RedisClient* redis_client) {

    redis_client->setEigen(CONTROL_POINT_POS, motion_force_task->getCurrentPosition());
    redis_client->setEigen(CONTROL_POINT_ORIENT, motion_force_task->getCurrentOrientation());
    redis_client->setEigen(CONTROL_POINT_VEL, motion_force_task->getCurrentLinearVelocity());
}

void update_smoothed_force_torque(VectorBuffer* buffer, SaiCommon::RedisClient* redis_client) {

    VectorXd sensed_force_torque(sensed_force.size() + sensed_torque.size());

    sensed_force_torque << sensed_force,  sensed_torque;

    VectorXd average_force_torque = buffer->addValue(sensed_force_torque);

    VectorXd average_force = average_force_torque.head(3);
    VectorXd average_torque = average_force_torque.tail(3);

    redis_client->setEigen(SMOOTHED_FORCE, average_force);
    redis_client->setEigen(SMOOTHED_TORQUE, average_torque);
}

void updateRobotState(std::shared_ptr<SaiModel::SaiModel> robot) {

   robot->setQ(robot_q);
   robot->setDq(robot_dq);
   robot->updateModel();

}

void compute_joint_torques(std::shared_ptr<SaiModel::SaiModel> robot, 
    std::shared_ptr<SaiPrimitives::MotionForceTask> motion_force_task, 
    std::shared_ptr<SaiPrimitives::JointTask> joint_task) {

    motion_force_task->setGoalForce(desired_force);
    int force_dim = static_cast<int>(motion_force_axis[3]);

    motion_force_task->parametrizeForceMotionSpaces(force_dim, Vector3d(motion_force_axis[0], motion_force_axis[1], motion_force_axis[2])); 
    motion_force_task->setGoalPosition(target_pos);
    Eigen::Quaterniond qd(static_cast<double>(target_orient[3]),
                      static_cast<double>(target_orient[0]),
                      static_cast<double>(target_orient[1]),
                      static_cast<double>(target_orient[2]));

    Matrix3d orient = qd.normalized().toRotationMatrix();
    motion_force_task->setGoalOrientation(orient);

    motion_force_task->updateTaskModel(MatrixXd::Identity(robot->dof(), robot->dof()));
    joint_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
    control_torques = motion_force_task->computeTorques() + joint_task->computeTorques();
}

int main(int argc, char** argv) {
	// Location of URDF files specifying world and robot information
	static const string robot_file = std::string(URDF_PATH) + "/panda_arm_gripper.urdf";

    auto force_torque_buffer = VectorBuffer(50, 6);

    auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

    init_keys(&redis_client);
    set_send_receive_keys(&redis_client);

    redis_client.receiveAllFromGroup();

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);

	updateRobotState(robot);

	auto dof = robot->dof();

    control_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.28);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto motion_force_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	motion_force_task->setPosControlGains(400, 40, 0);
	motion_force_task->setOriControlGains(400, 40, 0);

	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

    double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq, 1e6);

	runloop = true;
	int loop_count = 0;

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

        //recieve all the redis keys
        redis_client.receiveAllFromGroup();

		// update robot 
		updateRobotState(robot);

        update_smoothed_force_torque(&force_torque_buffer, &redis_client);

        updateControlPointValues(motion_force_task, &redis_client);

		// update task model
		N_prec.setIdentity();
		motion_force_task->updateTaskModel(N_prec);

        compute_joint_torques(robot, motion_force_task, joint_task);

        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);

		if (loop_count % 1000 == 0) {
			// std::cout << "control torques: " << control_torques << std::endl;
            std::cout << "Norm of distance between target point and control point: " << (target_pos - motion_force_task->getCurrentPosition()).norm() << std::endl;
            // std::cout << "Norm of distance between target point and control point: " <<  << std::endl;

            // std::cout << "sensed_force: " << sensed_force << std::endl;
		}

		loop_count += 1;

	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * control_torques);

	return 0;
}