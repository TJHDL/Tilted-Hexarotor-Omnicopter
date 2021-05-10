#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Eigen>
#include <mav_msgs/default_topics.h>
#include <omnicopter_controller/parameters_ros.h>
#include <omnicopter_controller/omnicopter_attitude_controller.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <sstream>

//Global variable
nav_msgs::Odometry AttitudeOdometryMsg;
ros::Publisher attitude_temp_n;

extern mav_msgs::EigenTrajectoryPointDeque attitude_commands;
extern std::deque<ros::Duration> attitude_command_waiting_times;
extern ros::Timer attitude_command_timer;

extern mav_msgs::EigenTrajectoryPoint attitude_command_trajectory;
extern Eigen::Vector3d desired_body_angular_velocity;
extern mav_msgs::EigenOdometry attitude_odometry_;

void AttitudePublishFunc(ros::Publisher& temp)
{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	// Todo(ffurrer): Do this in the conversions header.
	mav_msgs::RateThrustPtr angular_velocity_msg(new mav_msgs::RateThrust);

	for (int i = 0; i < desired_body_angular_velocity.size(); i++)
	{
		if(i == 0)
			angular_velocity_msg->angular_rates.x = desired_body_angular_velocity[i];
		else if(i == 1)
			angular_velocity_msg->angular_rates.y = desired_body_angular_velocity[i];
		else
			angular_velocity_msg->angular_rates.z = desired_body_angular_velocity[i];
	}
	//angular_velocity_msg.header.stamp = AttitudeOdometryMsg.header.stamp;

	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	temp.publish(angular_velocity_msg);

	// ROS_INFO("AttitudeController publish the desired angular velocity: [%f, %f, %f]."
	// 	, angular_velocity_msg->angular_rates.x
	// 	, angular_velocity_msg->angular_rates.y
	// 	, angular_velocity_msg->angular_rates.z);
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void CommandAttitudeCallback(const geometry_msgs::PoseStamped& pose_msg)
void CommandAttitudeCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& pose_msg)
{
	//ROS_INFO_ONCE("OmnicopterAttitudeController got first command pose message.");

	// Clear all pending commands.
	attitude_command_timer.stop();
	attitude_commands.clear();
	attitude_command_waiting_times.clear();

	const size_t n_commands = pose_msg->points.size();

	if(n_commands < 1)
	{
		ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
		return;
    }

	mav_msgs::EigenTrajectoryPoint eigen_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(pose_msg->points.front(), &eigen_reference);
	attitude_commands.push_front(eigen_reference);

	for (size_t i = 1; i < n_commands; ++i) 
	{
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = pose_msg->points[i-1];
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = pose_msg->points[i];

		mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

		attitude_commands.push_back(eigen_reference);
		attitude_command_waiting_times.push_back(current_reference.time_from_start - reference_before.time_from_start);
    }

	SetAttitudeTrajectoryPoint(attitude_commands.front());
	attitude_commands.pop_front();
	
	if (n_commands > 1) 
	{
		attitude_command_timer.setPeriod(attitude_command_waiting_times.front());
		attitude_command_waiting_times.pop_front();
		attitude_command_timer.start();
    }

	ROS_INFO("AttitudeController get the command attitude: [%f, %f, %f, %f]."
		, attitude_command_trajectory.orientation_W_B.x()
		, attitude_command_trajectory.orientation_W_B.y()
		, attitude_command_trajectory.orientation_W_B.z()
		, attitude_command_trajectory.orientation_W_B.w());
}

void AttitudeControllerCallback(const nav_msgs::Odometry& odometry_msg)
{
	//ROS_INFO_ONCE("OmnicopterAttitudeController got first odometry message.");

	mav_msgs::EigenOdometry odometry;
	mav_msgs::eigenOdometryFromMsg(odometry_msg, &odometry);
	SetAttitudeOdometry(odometry);

	AttitudeOdometryMsg = odometry_msg;

	// ROS_INFO("AttitudeController get the odometry attitude: [%f, %f, %f, %f]."
	// 	, attitude_odometry_.orientation_W_B.x()
	// 	, attitude_odometry_.orientation_W_B.y()
	// 	, attitude_odometry_.orientation_W_B.z()
	// 	, attitude_odometry_.orientation_W_B.w());

	CalculateDesiredBodyAngularVelocity(&desired_body_angular_velocity);

	// ROS_INFO("AttitudeController calculate the desired angular velocity: [%f, %f, %f]."
	// 		, desired_body_angular_velocity.x()
	// 		, desired_body_angular_velocity.y()
	// 		, desired_body_angular_velocity.z());

	AttitudePublishFunc(attitude_temp_n);
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
	ros::init(argc, argv, "attitude_controller");

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
	ros::Publisher chatter_pub = n.advertise<mav_msgs::RateThrust>("DesiredBodyAngularVelocity", 1);
	attitude_temp_n = chatter_pub;

	// Initialize the initial point.
	// attitude_command_trajectory.orientation_W_B.x() = 0;
	// attitude_command_trajectory.orientation_W_B.y() = 0;
	// attitude_command_trajectory.orientation_W_B.z() = 0;
	// attitude_command_trajectory.orientation_W_B.w() = 1;

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
	attitude_command_timer = n.createTimer(ros::Duration(0), AttitudeTimedCommandCallback);
	ros::Subscriber cmd_attitude_sub = n.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, CommandAttitudeCallback);
	ros::Subscriber attitude_odometry_sub = n.subscribe(mav_msgs::default_topics::ODOMETRY, 1, AttitudeControllerCallback);
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}