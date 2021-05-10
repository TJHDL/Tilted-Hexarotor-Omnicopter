#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Eigen>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/Vector3.h>
#include <omnicopter_controller/parameters_ros.h>
#include <omnicopter_controller/omnicopter_body_angular_velocity_controller.h>

#include <sstream>

//Global variable
nav_msgs::Odometry AngularVelocityOdometryMsg;
ros::Publisher angular_velocity_temp_n;

extern Eigen::Vector3d desired_body_total_torque;
extern mav_msgs::EigenRateThrust body_angular_velocity_;
extern mav_msgs::EigenOdometry angular_velocity_odometry_;

void AngularVelocityPublishFunc(ros::Publisher& temp)
{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	// Todo(ffurrer): Do this in the conversions header.
	mav_msgs::TorqueThrustPtr torque_msg(new mav_msgs::TorqueThrust);

	for (int i = 0; i < desired_body_total_torque.size(); i++)
	{
		if(i == 0)
			torque_msg->torque.x = desired_body_total_torque[i];
		else if(i == 1)
			torque_msg->torque.y = desired_body_total_torque[i];
		else
			torque_msg->torque.z = desired_body_total_torque[i];
	}
	//torque_msg.header.stamp = AngularVelocityOdometryMsg.header.stamp;

	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	temp.publish(torque_msg);

	// ROS_INFO("AttitudeController publish the desired total torque: [%f, %f, %f]."
	// 	, torque_msg->torque.x
	// 	, torque_msg->torque.y
	// 	, torque_msg->torque.z);
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void CommandAngularVelocityCallback(const mav_msgs::RateThrust& angular_velocity_msg)
{
	//ROS_INFO_ONCE("OmnicopterBodyAngularVelocityController got first command ratethrust message.");

	mav_msgs::EigenRateThrust body_angular_velocity;
	mav_msgs::eigenRateThrustFromMsg(angular_velocity_msg, &body_angular_velocity);
	SetAngularVelocity(body_angular_velocity);

	// ROS_INFO("BodyAngularVelocityController get the command angular velocity: [%f, %f, %f]."
	// 	, body_angular_velocity_.angular_rates.x()
	// 	, body_angular_velocity_.angular_rates.y()
	// 	, body_angular_velocity_.angular_rates.z());
}

void AngularVelocityControllerCallback(const nav_msgs::Odometry& odometry_msg)
{
	//ROS_INFO_ONCE("OmnicopterBodyAngularVelocityController got first odometry message.");

	mav_msgs::EigenOdometry odometry;
	mav_msgs::eigenOdometryFromMsg(odometry_msg, &odometry);
	SetAngularVelocityOdometry(odometry);

	AngularVelocityOdometryMsg = odometry_msg;

	// ROS_INFO("BodyAngularVelocityController get the odometry angular velocity: [%f, %f, %f]."
	// 	, angular_velocity_odometry_.angular_velocity_B.x()
	// 	, angular_velocity_odometry_.angular_velocity_B.y()
	// 	, angular_velocity_odometry_.angular_velocity_B.z());

	CalculateDesiredTotalTorque(&desired_body_total_torque);

	AngularVelocityPublishFunc(angular_velocity_temp_n);
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
	ros::init(argc, argv, "body_angular_velocity_controller");

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
	ros::Publisher chatter_pub = n.advertise<mav_msgs::TorqueThrust>("DesiredTorque", 1);
	angular_velocity_temp_n = chatter_pub;
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
	ros::Subscriber cmd_angular_velocity_sub = n.subscribe("DesiredBodyAngularVelocity", 1, CommandAngularVelocityCallback);
	ros::Subscriber angular_velocity_odometry_sub = n.subscribe(mav_msgs::default_topics::ODOMETRY, 1, AngularVelocityControllerCallback);
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}