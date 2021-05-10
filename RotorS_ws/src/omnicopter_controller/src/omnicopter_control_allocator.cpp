#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Eigen>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/Vector3.h>
#include <omnicopter_controller/parameters_ros.h>
#include <omnicopter_controller/omnicopter_control_allocator.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <sstream>

//Global variable
extern bool take_off_active;

mav_msgs::TorqueThrust ThrustTorqueMsg;
ros::Publisher control_allocator_temp_n;

extern mav_msgs::EigenTrajectoryPointDeque allocator_commands;
extern std::deque<ros::Duration> allocator_command_waiting_times;
extern ros::Timer allocator_command_timer;

extern mav_msgs::EigenTrajectoryPoint allocator_command_trajectory;
extern Eigen::VectorXd desired_rotors_velocity;
extern mav_msgs::EigenTorqueThrust thrust_;
extern mav_msgs::EigenTorqueThrust torque_;

void ControlAllocatorPublishFunc(ros::Publisher& temp)
{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	// Todo(ffurrer): Do this in the conversions header.
	mav_msgs::ActuatorsPtr velocity_msg(new mav_msgs::Actuators);

	velocity_msg->angular_velocities.clear();
	for (int i = 0; i < desired_rotors_velocity.size(); i++)
		velocity_msg->angular_velocities.push_back(desired_rotors_velocity[i]);
	//velocity_msg.header.stamp = ThrustTorqueMsg.header.stamp;

	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	temp.publish(velocity_msg);

	// ROS_INFO("ControlAllocator publish the desired rotor velocity: [%f, %f, %f, %f, %f, %f]."
	// 	, desired_rotors_velocity[0]
	// 	, desired_rotors_velocity[1]
	// 	, desired_rotors_velocity[2]
	// 	, desired_rotors_velocity[3]
	// 	, desired_rotors_velocity[4]
	// 	, desired_rotors_velocity[5]);
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void CommandPoseCallback(const geometry_msgs::PoseStamped& pose_msg)
void CommandAllocatorCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& pose_msg)
{
	//ROS_INFO_ONCE("OmnicopterAllocatorController got first command pose message.");

	// Clear all pending commands.
	allocator_command_timer.stop();
	allocator_commands.clear();
	allocator_command_waiting_times.clear();

	const size_t n_commands = pose_msg->points.size();

	if(n_commands < 1)
	{
		ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
		return;
    }

	mav_msgs::EigenTrajectoryPoint eigen_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(pose_msg->points.front(), &eigen_reference);
	allocator_commands.push_front(eigen_reference);

	for (size_t i = 1; i < n_commands; ++i) 
	{
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = pose_msg->points[i-1];
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = pose_msg->points[i];

		mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

		allocator_commands.push_back(eigen_reference);
		allocator_command_waiting_times.push_back(current_reference.time_from_start - reference_before.time_from_start);
    }

	SetAllocatorTrajectoryPoint(allocator_commands.front());
	allocator_commands.pop_front();
	
	if (n_commands > 1) 
	{
		allocator_command_timer.setPeriod(allocator_command_waiting_times.front());
		allocator_command_waiting_times.pop_front();
		allocator_command_timer.start();
    }

	// ROS_INFO("AllocatorController get the command position: [%f, %f, %f]."
	// 	, allocator_command_trajectory.position_W.x()
	// 	, allocator_command_trajectory.position_W.y()
	// 	, allocator_command_trajectory.position_W.z());
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void ThrustCallback(const mav_msgs::TorqueThrust& thrust_msg)
{
	//ROS_INFO_ONCE("OmnicopterControlAllocator got first command torquethrust message.");

	mav_msgs::EigenTorqueThrust thrust;
	mav_msgs::eigenTorqueThrustFromMsg(thrust_msg, &thrust);
	SetThrust(thrust);

	// ROS_INFO("ControlAllocator get the command thrust: [%f, %f, %f]."
	// 	, thrust_.thrust.x()
	// 	, thrust_.thrust.y()
	// 	, thrust_.thrust.z());
}

void TorqueCallback(const mav_msgs::TorqueThrust& torque_msg)
{
	mav_msgs::EigenTorqueThrust torque;
	mav_msgs::eigenTorqueThrustFromMsg(torque_msg, &torque);
	SetTorque(torque);

	ThrustTorqueMsg = torque_msg;

	// ROS_INFO("ControlAllocator get the command torque: [%f, %f, %f]."
	// 	, torque_.torque.x()
	// 	, torque_.torque.y()
	// 	, torque_.torque.z());

	CalculateDesiredRotorsVelocity(thrust_, torque_, &desired_rotors_velocity);

	if((allocator_command_trajectory.position_W.x() != 0) && (allocator_command_trajectory.position_W.x() != NULL))
		take_off_active = true;
	else if((allocator_command_trajectory.position_W.y() != 0) && (allocator_command_trajectory.position_W.y() != NULL))
		take_off_active = true;
	else if((allocator_command_trajectory.position_W.z() != 0) && (allocator_command_trajectory.position_W.z() != NULL))
		take_off_active = true;

	if(take_off_active)
		ControlAllocatorPublishFunc(control_allocator_temp_n);
}

int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "omnicopter_control_allocator");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher chatter_pub = n.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
	control_allocator_temp_n = chatter_pub;
	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	allocator_command_timer = n.createTimer(ros::Duration(0), AllocatorTimedCommandCallback);
	ros::Subscriber cmd_allocator_sub = n.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, CommandAllocatorCallback);
	ros::Subscriber allocator_thrust_sub = n.subscribe("DesiredThrust", 1, ThrustCallback);
	ros::Subscriber allocator_torque_sub = n.subscribe("DesiredTorque", 1, TorqueCallback);
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}