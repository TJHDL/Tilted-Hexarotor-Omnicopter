#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Eigen>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/Vector3.h>
#include <omnicopter_controller/parameters_ros.h>
#include <omnicopter_controller/omnicopter_position_controller.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <sstream>

//Global variable
nav_msgs::Odometry PositionOdometryMsg;
ros::Publisher position_temp_n;

extern mav_msgs::EigenTrajectoryPointDeque position_commands;
extern std::deque<ros::Duration> position_command_waiting_times;
extern ros::Timer position_command_timer;

extern mav_msgs::EigenTrajectoryPoint position_command_trajectory;
extern Eigen::Vector3d desired_body_total_f;
extern mav_msgs::EigenOdometry position_odometry_;

void PositionPublishFunc(ros::Publisher& temp)
{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	// Todo(ffurrer): Do this in the conversions header.
	mav_msgs::TorqueThrustPtr thrust_msg(new mav_msgs::TorqueThrust);

	for (int i = 0; i < desired_body_total_f.size(); i++)
	{
		if(i == 0)
			thrust_msg->thrust.x = desired_body_total_f[i];
		else if(i == 1)
			thrust_msg->thrust.y = desired_body_total_f[i];
		else
			thrust_msg->thrust.z = desired_body_total_f[i];
	}
	//thrust_msg.header.stamp = PositionOdometryMsg.header.stamp;

	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	temp.publish(thrust_msg);

	// ROS_INFO("AttitudeController publish the desired angular velocity: [%f, %f, %f]."
	// 	, angular_velocity_msg->angular_rates.x
	// 	, angular_velocity_msg->angular_rates.y
	// 	, angular_velocity_msg->angular_rates.z);
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void CommandPoseCallback(const geometry_msgs::PoseStamped& pose_msg)
void CommandPositionCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& pose_msg)
{
	//ROS_INFO_ONCE("OmnicopterPositionController got first command pose message.");

	// Clear all pending commands.
	position_command_timer.stop();
	position_commands.clear();
	position_command_waiting_times.clear();

	const size_t n_commands = pose_msg->points.size();

	if(n_commands < 1)
	{
		ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
		return;
    }

	mav_msgs::EigenTrajectoryPoint eigen_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(pose_msg->points.front(), &eigen_reference);
	position_commands.push_front(eigen_reference);

	for (size_t i = 1; i < n_commands; ++i) 
	{
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = pose_msg->points[i-1];
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = pose_msg->points[i];

		mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

		position_commands.push_back(eigen_reference);
		position_command_waiting_times.push_back(current_reference.time_from_start - reference_before.time_from_start);
    }

	SetPositionTrajectoryPoint(position_commands.front());
	position_commands.pop_front();
	
	if (n_commands > 1) 
	{
		position_command_timer.setPeriod(position_command_waiting_times.front());
		position_command_waiting_times.pop_front();
		position_command_timer.start();
    }

	ROS_INFO("PositionController get the command position: [%f, %f, %f]."
		, position_command_trajectory.position_W.x()
		, position_command_trajectory.position_W.y()
		, position_command_trajectory.position_W.z());
}

void PositionControllerCallback(const nav_msgs::Odometry& odometry_msg)
{
	//ROS_INFO_ONCE("OmnicopterPositionController got first odometry message.");

	mav_msgs::EigenOdometry odometry;
	mav_msgs::eigenOdometryFromMsg(odometry_msg, &odometry);
	SetPositionOdometry(odometry);

	PositionOdometryMsg = odometry_msg;

	// ROS_INFO("PositionController get the odometry position: [%f, %f, %f]."
	// 	, position_odometry_.position_W.x()
	// 	, position_odometry_.position_W.y()
	// 	, position_odometry_.position_W.z());

	CalculateDesiredBodyTotalThrust(position_odometry_, &desired_body_total_f);

	PositionPublishFunc(position_temp_n);
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
	ros::init(argc, argv, "position_controller");

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
	ros::Publisher chatter_pub = n.advertise<mav_msgs::TorqueThrust>("DesiredThrust", 1);
	position_temp_n = chatter_pub;

	// Initialize the initial point.
	// position_command_trajectory.position_W.x() = 0;
	// position_command_trajectory.position_W.y() = 0;
	// position_command_trajectory.position_W.z() = 0;
	
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
	position_command_timer = n.createTimer(ros::Duration(0), PositionTimedCommandCallback);
	ros::Subscriber cmd_pose_sub = n.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, CommandPositionCallback);
	ros::Subscriber position_odometry_sub = n.subscribe(mav_msgs::default_topics::ODOMETRY, 1, PositionControllerCallback);
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}