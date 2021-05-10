#include "stdio.h"
#include "cmath"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include "omnicopter_controller/common.h"
#include "omnicopter_controller/parameters.h"

#define kpp 1
#define kpi 0.01
#define kpd 100

//Global variable
mav_msgs::EigenTrajectoryPointDeque position_commands;
std::deque<ros::Duration> position_command_waiting_times;
ros::Timer position_command_timer;

mav_msgs::EigenTrajectoryPoint position_command_trajectory;
mav_msgs::EigenOdometry position_odometry_;
Eigen::Vector3d desired_body_total_f;
rotors_control::VehicleParameters vehicle_parameters;
Eigen::Matrix3d rotation_matrix;
Eigen::Vector3d desired_acceleration;
Eigen::Vector3d integrated_position_error = Eigen::Vector3d(0, 0, 0);
double previous_position_odometry_timestamp = 0;

// Default values for PID controller.
static const Eigen::Vector3d k_p_p = Eigen::Vector3d(kpp, kpp, kpp);
static const Eigen::Vector3d k_p_i = Eigen::Vector3d(kpi, kpi, kpi);
static const Eigen::Vector3d k_p_d = Eigen::Vector3d(kpd, kpd, kpd);

void SetPositionTrajectoryPoint(mav_msgs::EigenTrajectoryPoint& command_trajectory1) 
{
    position_command_trajectory = command_trajectory1;
}

void SetPositionOdometry(mav_msgs::EigenOdometry& odometry1) 
{
    position_odometry_ = odometry1;
}

void PositionTimedCommandCallback(const ros::TimerEvent& e) 
{
  if(position_commands.empty())
  {
    // ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = position_commands.front();
  SetPositionTrajectoryPoint(position_commands.front());
  position_commands.pop_front();
  position_command_timer.stop();
  if(!position_command_waiting_times.empty())
  {
    position_command_timer.setPeriod(position_command_waiting_times.front());
    position_command_waiting_times.pop_front();
    position_command_timer.start();
  }
}

void CalculateRotationMatrix(mav_msgs::EigenOdometry& odometry)
{
    rotation_matrix = odometry.orientation_W_B.toRotationMatrix();
}

void CalculateDesiredAcceleration(Eigen::Vector3d* acceleration)
{
    bool GetToTheCommandPoint = true;

    Eigen::Vector3d position_error;
    position_error = position_odometry_.position_W - position_command_trajectory.position_W;
    position_error.z() -= 0.1;

    integrated_position_error += position_error * (double(position_odometry_.timestamp_ns) / 1.000e9 - previous_position_odometry_timestamp);
    previous_position_odometry_timestamp = position_odometry_.timestamp_ns / 1.000e9;

    // for(int i = 0; i < integrated_position_error.size(); i++)
    // {
    //     if(abs(integrated_position_error[i]) >= 100)
    //     {
    //         if(integrated_position_error[i] <= -100)
    //             integrated_position_error[i] = -100;
    //         else
    //             integrated_position_error[i] = 100;
    //     }
    // }

    // Transform velocity to world frame.
    Eigen::Vector3d velocity_W =  rotation_matrix * position_odometry_.velocity_B;
    Eigen::Vector3d velocity_error;
    velocity_error = velocity_W - position_command_trajectory.velocity_W;

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

    *acceleration = (-position_error.cwiseProduct(k_p_p)) + (-velocity_error.cwiseProduct(k_p_d))
       + (-integrated_position_error.cwiseProduct(k_p_i))
        + vehicle_parameters.gravity_ * e_3 + position_command_trajectory.acceleration_W;

    // if(abs(position_command_trajectory.position_W[0] - position_odometry_.position_W[0]) >= 0.3)
    //     GetToTheCommandPoint = false;
    // else if(abs(position_command_trajectory.position_W[1] - position_odometry_.position_W[1]) >= 0.4)
    //     GetToTheCommandPoint = false;
    // else if(abs(position_command_trajectory.position_W[2] - position_odometry_.position_W[2]) >= 0.2)
    //     GetToTheCommandPoint = false;
    if(abs(position_command_trajectory.position_W[2] - position_odometry_.position_W[2]) >= 0.2)
        GetToTheCommandPoint = false;

    if(GetToTheCommandPoint)
    {
        integrated_position_error = Eigen::Vector3d(0, 0, 0);
        *acceleration = vehicle_parameters.gravity_ * e_3 + position_command_trajectory.acceleration_W;
    }

    // ROS_INFO("PositionController calculated the rotation matrix: [%f, %f, %f, %f, %f, %f, %f, %f, %f]."
	// 	, rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2)
	// 	, rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2)
	// 	, rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

    // ROS_INFO("PositionController calculated the acceleration: [%f, %f, %f]."
    //     , acceleration->x()
    //     , acceleration->y()
    //     , acceleration->z());

    // ROS_INFO("PositionController calculated the position error: [%f, %f, %f]."
    //     , integrated_position_error[0]
    //     , integrated_position_error[1]
    //     , integrated_position_error[2]);

    // ROS_INFO("PositionController got the position: [%f, %f, %f]."
    //     , position_odometry_.position_W[0]
    //     , position_odometry_.position_W[1]
    //     , position_odometry_.position_W[2]);
}

void CalculateDesiredBodyTotalThrust(mav_msgs::EigenOdometry& odometry, Eigen::Vector3d* desired_body_total_thrust)
{
     CalculateRotationMatrix(odometry);
     CalculateDesiredAcceleration(&desired_acceleration);

     Eigen::MatrixXd mav_mass(3,1);
     mav_mass << vehicle_parameters.mass_, vehicle_parameters.mass_, vehicle_parameters.mass_;
     
     Eigen::MatrixXd result(3,1);
     result = (rotation_matrix * desired_acceleration).cwiseProduct(mav_mass);

     *desired_body_total_thrust << result(0, 0), result(1, 0), result(2, 0);

    //  ROS_INFO("PositionController calculated the desired body total thrust: [%f, %f, %f]."
	// 	, desired_body_total_thrust->x()
	// 	, desired_body_total_thrust->y()
	// 	, desired_body_total_thrust->z());
}