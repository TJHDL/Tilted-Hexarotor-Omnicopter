#include "stdio.h"
#include "cmath"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include "omnicopter_controller/common.h"
#include "omnicopter_controller/parameters.h"

#define kap 1
#define kai 0.01

//Global variable
mav_msgs::EigenTrajectoryPointDeque attitude_commands;
std::deque<ros::Duration> attitude_command_waiting_times;
ros::Timer attitude_command_timer;

mav_msgs::EigenTrajectoryPoint attitude_command_trajectory;
mav_msgs::EigenOdometry attitude_odometry_;
Eigen::Matrix3d attitude_error_rotation_inverse_matrix;
Eigen::Vector3d desired_body_angular_velocity;
Eigen::Vector3d integrated_attitude_virtual_part_error = Eigen::Vector3d(0, 0, 0);
double previous_attitude_odometry_timestamp = 0;

// Default values for PID controller.
static const Eigen::Vector3d k_a_p = Eigen::Vector3d(kap, kap, kap);
static const Eigen::Vector3d k_a_i = Eigen::Vector3d(kai, kai, kai);

void SetAttitudeTrajectoryPoint(mav_msgs::EigenTrajectoryPoint& command_trajectory1) 
{
    attitude_command_trajectory = command_trajectory1;
}

void SetAttitudeOdometry(mav_msgs::EigenOdometry& odometry1) 
{
    attitude_odometry_ = odometry1;
}

void AttitudeTimedCommandCallback(const ros::TimerEvent& e) 
{
  if(attitude_commands.empty())
  {
    // ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = attitude_commands.front();
  SetAttitudeTrajectoryPoint(attitude_commands.front());
  attitude_commands.pop_front();
  attitude_command_timer.stop();
  if(!attitude_command_waiting_times.empty())
  {
    attitude_command_timer.setPeriod(attitude_command_waiting_times.front());
    attitude_command_waiting_times.pop_front();
    attitude_command_timer.start();
  }
}

void CalculateAttitudeErrorRotationInverseMatrix(Eigen::Quaterniond& attitude_error_quaternion)
{
    attitude_error_rotation_inverse_matrix = attitude_error_quaternion.toRotationMatrix().inverse();
}


void CalculateDesiredBodyAngularVelocity(Eigen::Vector3d* desired_angular_velocity)
{
    bool GetToTheCommandAttitude = true;

    Eigen::Quaterniond attitude_error;
    attitude_error.x() = attitude_odometry_.orientation_W_B.x() - attitude_command_trajectory.orientation_W_B.x();
    attitude_error.y() = attitude_odometry_.orientation_W_B.y() - attitude_command_trajectory.orientation_W_B.y();
    attitude_error.z() = attitude_odometry_.orientation_W_B.z() - attitude_command_trajectory.orientation_W_B.z();
    attitude_error.w() = attitude_odometry_.orientation_W_B.w() - attitude_command_trajectory.orientation_W_B.w();

    Eigen::Vector3d attitude_virtual_part_error;
    attitude_virtual_part_error = Eigen::Vector3d(attitude_error.x(), attitude_error.y(), attitude_error.z());

    integrated_attitude_virtual_part_error += attitude_virtual_part_error * (double(attitude_odometry_.timestamp_ns) / 1.000e9 - previous_attitude_odometry_timestamp);
    previous_attitude_odometry_timestamp = attitude_odometry_.timestamp_ns / 1.000e9;

    // for(int i = 0; i < integrated_attitude_virtual_part_error.size(); i++)
    // {
    //     if(abs(integrated_attitude_virtual_part_error[i]) >= 100)
    //     {
    //         if(integrated_attitude_virtual_part_error[i] <= -100)
    //             integrated_attitude_virtual_part_error[i] = -100;
    //         else
    //             integrated_attitude_virtual_part_error[i] = 100;
    //     }
    // }

    CalculateAttitudeErrorRotationInverseMatrix(attitude_error);

    *desired_angular_velocity = (-attitude_virtual_part_error.cwiseProduct(k_a_p)) + (-integrated_attitude_virtual_part_error.cwiseProduct(k_a_i))
        + attitude_error_rotation_inverse_matrix * attitude_odometry_.angular_velocity_B;

    for(int i = 0; i < 4; i++)
    {
        if(i == 0)
        {
            if(abs(attitude_command_trajectory.orientation_W_B.x() - attitude_odometry_.orientation_W_B.x()) >= 0.005)
                GetToTheCommandAttitude = false;
        }
        else if(i == 1)
        {
            if(abs(attitude_command_trajectory.orientation_W_B.y() - attitude_odometry_.orientation_W_B.y()) >= 0.005)
                GetToTheCommandAttitude = false;
        }
        else if(i == 2)
        {
            if(abs(attitude_command_trajectory.orientation_W_B.z() - attitude_odometry_.orientation_W_B.z()) >= 0.005)
                GetToTheCommandAttitude = false;
        }
        else
        {
            if(abs(attitude_command_trajectory.orientation_W_B.w() - attitude_odometry_.orientation_W_B.w()) >= 0.005)
                GetToTheCommandAttitude = false;
        }
    }

    if(GetToTheCommandAttitude)
    {
        integrated_attitude_virtual_part_error = Eigen::Vector3d(0, 0, 0);
        (*desired_angular_velocity)[0] = 0;
        (*desired_angular_velocity)[1] = 0;
        (*desired_angular_velocity)[2] = 0;
    }

    // ROS_INFO("AttitudeController calculated the desired angular velocity: [%f, %f, %f]."
	// 	, (*desired_angular_velocity)[0]
	// 	, (*desired_angular_velocity)[1]
	// 	, (*desired_angular_velocity)[2]);

    // ROS_INFO("AttitudeController calculated the attitude error: [%f, %f, %f]."
	// 	, integrated_attitude_virtual_part_error[0]
	// 	, integrated_attitude_virtual_part_error[1]
	// 	, integrated_attitude_virtual_part_error[2]);

    // ROS_INFO("AttitudeController got the attitude: [%f, %f, %f, %f]."
	// 	, attitude_odometry_.orientation_W_B.x()
	// 	, attitude_odometry_.orientation_W_B.y()
	// 	, attitude_odometry_.orientation_W_B.z()
    //     , attitude_odometry_.orientation_W_B.w());


    // ROS_INFO("The odometry's timestamp: %ld.", attitude_odometry_.timestamp_ns);
}