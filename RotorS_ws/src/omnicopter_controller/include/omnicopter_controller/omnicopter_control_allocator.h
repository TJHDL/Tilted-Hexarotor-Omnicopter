#include "stdio.h"
#include "cmath"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include "omnicopter_controller/common.h"
#include "omnicopter_controller/parameters.h"

//Defalt vehicle parameter
#define tilt_angle 30*3.1415926/180
#define rod_length 0.6
#define cf 1.253e-05    //thrust constant
#define ct 1.699e-07    //torque constant
#define rotor_velocity_max 1000.00  //max rotor's velocity
#define rotor_velocity_min 0.00    //min rotor's velocity

//Global variable
bool take_off_active = false;

mav_msgs::EigenTrajectoryPointDeque allocator_commands;
std::deque<ros::Duration> allocator_command_waiting_times;
ros::Timer allocator_command_timer;

mav_msgs::EigenTrajectoryPoint allocator_command_trajectory;
mav_msgs::EigenTorqueThrust thrust_;
mav_msgs::EigenTorqueThrust torque_;
Eigen::VectorXd desired_allocator_vector(6);
Eigen::VectorXd desired_rotors_velocity(6);
Eigen::MatrixXd position_matrix(3,6);
Eigen::MatrixXd orientation_matrix(3,6);

void SetThrust(mav_msgs::EigenTorqueThrust& thrust1)
{
    thrust_ = thrust1;
}

void SetTorque(mav_msgs::EigenTorqueThrust& torque1)
{
    torque_ = torque1;
}

void SetAllocatorTrajectoryPoint(mav_msgs::EigenTrajectoryPoint& command_trajectory1) 
{
    allocator_command_trajectory = command_trajectory1;
}

void AllocatorTimedCommandCallback(const ros::TimerEvent& e) 
{
  if(allocator_commands.empty())
  {
    // ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = allocator_commands.front();
  SetAllocatorTrajectoryPoint(allocator_commands.front());
  allocator_commands.pop_front();
  allocator_command_timer.stop();
  if(!allocator_command_waiting_times.empty())
  {
    allocator_command_timer.setPeriod(allocator_command_waiting_times.front());
    allocator_command_waiting_times.pop_front();
    allocator_command_timer.start();
  }
}

void CalculateDesiredAllocatorVector(mav_msgs::EigenTorqueThrust& desired_thrust
    , mav_msgs::EigenTorqueThrust& desired_torque
    , Eigen::VectorXd* allocator_vector)
{
    position_matrix << rod_length, 0.5*rod_length, -0.5*rod_length, -1*rod_length, -0.5*rod_length, 0.5*rod_length,
                            0, sqrt(3)/2*rod_length, sqrt(3)/2*rod_length, 0, -1*sqrt(3)/2*rod_length, -1*sqrt(3)/2*rod_length,
                            0, 0, 0, 0, 0, 0;
                                    
    orientation_matrix << 0, -1*sqrt(3)/2*sin(tilt_angle), sqrt(3)/2*sin(tilt_angle), 0, -1*sqrt(3)/2*sin(tilt_angle), sqrt(3)/2*sin(tilt_angle),
                        -1*sin(tilt_angle), 0.5*sin(tilt_angle), 0.5*sin(tilt_angle), -1*sin(tilt_angle), 0.5*sin(tilt_angle), 0.5*sin(tilt_angle),
                        cos(tilt_angle), cos(tilt_angle), cos(tilt_angle), cos(tilt_angle), cos(tilt_angle), cos(tilt_angle);

    Eigen::Vector3d fd, td;
    fd = desired_thrust.thrust;
    td = desired_torque.torque;

    Eigen::VectorXd desired_thrust_torque(6);
    desired_thrust_torque << fd.x(), fd.y(), fd.z(), td.x(), td.y(), td.z();

    Eigen::MatrixXd M_Matrix(6,6), M_Pseude_Inverse_Matrix(6,6);
    Eigen::MatrixXd P_N_Cross_Matrix(3,6);

    for(int i = 0; i < 6; i++)
    {
        Eigen::Vector3d Position_Matrix_I_ColVector, Orientation_Matrix_I_ColVector;
        Position_Matrix_I_ColVector << position_matrix.col(i);
        Orientation_Matrix_I_ColVector << orientation_matrix.col(i);
        P_N_Cross_Matrix.col(i) = Position_Matrix_I_ColVector.cross(Orientation_Matrix_I_ColVector);
    }

    M_Matrix << orientation_matrix, P_N_Cross_Matrix;
    M_Pseude_Inverse_Matrix = M_Matrix.transpose() * (M_Matrix * M_Matrix.transpose()).inverse();

    Eigen::MatrixXd result(6,1);
    result = M_Pseude_Inverse_Matrix * desired_thrust_torque;

    *allocator_vector << result(0, 0), result(1, 0), result(2, 0), result(3, 0), result(4, 0), result(5, 0);

    // ROS_INFO("ControlAllocator calculate the desired thrust and torque: [%f, %f, %f, %f, %f, %f]."
	// 	, desired_thrust_torque[0]
	// 	, desired_thrust_torque[1]
	// 	, desired_thrust_torque[2]
	// 	, desired_thrust_torque[3]
	// 	, desired_thrust_torque[4]
	// 	, desired_thrust_torque[5]);

    // ROS_INFO("ControlAllocator calculate the allocator vector: [%f, %f, %f, %f, %f, %f]."
	// 	, (*allocator_vector)[0]
	// 	, (*allocator_vector)[1]
	// 	, (*allocator_vector)[2]
	// 	, (*allocator_vector)[3]
	// 	, (*allocator_vector)[4]
	// 	, (*allocator_vector)[5]);

    // ROS_INFO("ControlAllocator calculate the pseude inverse matrix:[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]."
	// 	, M_Pseude_Inverse_Matrix(0,0), M_Pseude_Inverse_Matrix(0,1), M_Pseude_Inverse_Matrix(0,2), M_Pseude_Inverse_Matrix(0,3), M_Pseude_Inverse_Matrix(0,4), M_Pseude_Inverse_Matrix(0,5)
	// 	, M_Pseude_Inverse_Matrix(1,0), M_Pseude_Inverse_Matrix(1,1), M_Pseude_Inverse_Matrix(1,2), M_Pseude_Inverse_Matrix(1,3), M_Pseude_Inverse_Matrix(1,4), M_Pseude_Inverse_Matrix(1,5)
	// 	, M_Pseude_Inverse_Matrix(2,0), M_Pseude_Inverse_Matrix(2,1), M_Pseude_Inverse_Matrix(2,2), M_Pseude_Inverse_Matrix(2,3), M_Pseude_Inverse_Matrix(2,4), M_Pseude_Inverse_Matrix(2,5)
	// 	, M_Pseude_Inverse_Matrix(3,0), M_Pseude_Inverse_Matrix(3,1), M_Pseude_Inverse_Matrix(3,2), M_Pseude_Inverse_Matrix(3,3), M_Pseude_Inverse_Matrix(3,4), M_Pseude_Inverse_Matrix(3,5)
	// 	, M_Pseude_Inverse_Matrix(4,0), M_Pseude_Inverse_Matrix(4,1), M_Pseude_Inverse_Matrix(4,2), M_Pseude_Inverse_Matrix(4,3), M_Pseude_Inverse_Matrix(4,4), M_Pseude_Inverse_Matrix(4,5)
	// 	, M_Pseude_Inverse_Matrix(5,0), M_Pseude_Inverse_Matrix(5,1), M_Pseude_Inverse_Matrix(5,2), M_Pseude_Inverse_Matrix(5,3), M_Pseude_Inverse_Matrix(5,4), M_Pseude_Inverse_Matrix(5,5));
    
    // ROS_INFO("ControlAllocator calculate the M matrix:[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]."
	// 	, M_Matrix(0,0), M_Matrix(0,1), M_Matrix(0,2), M_Matrix(0,3), M_Matrix(0,4), M_Matrix(0,5)
	// 	, M_Matrix(1,0), M_Matrix(1,1), M_Matrix(1,2), M_Matrix(1,3), M_Matrix(1,4), M_Matrix(1,5)
	// 	, M_Matrix(2,0), M_Matrix(2,1), M_Matrix(2,2), M_Matrix(2,3), M_Matrix(2,4), M_Matrix(2,5)
	// 	, M_Matrix(3,0), M_Matrix(3,1), M_Matrix(3,2), M_Matrix(3,3), M_Matrix(3,4), M_Matrix(3,5)
	// 	, M_Matrix(4,0), M_Matrix(4,1), M_Matrix(4,2), M_Matrix(4,3), M_Matrix(4,4), M_Matrix(4,5)
	// 	, M_Matrix(5,0), M_Matrix(5,1), M_Matrix(5,2), M_Matrix(5,3), M_Matrix(5,4), M_Matrix(5,5));
}

void CalculateDesiredRotorsVelocity(mav_msgs::EigenTorqueThrust& desired_thrust
    , mav_msgs::EigenTorqueThrust& desired_torque
    , Eigen::VectorXd* rotors_velocity)
{
    CalculateDesiredAllocatorVector(desired_thrust, desired_torque, &desired_allocator_vector);

    Eigen::VectorXd rotor_velocity_temp(6);
    for (int i = 0; i < 6; i++)
    {
        if(i >= 0 && i < 3)
        {
            if(desired_allocator_vector[i] < rotor_velocity_min)
                rotor_velocity_temp[i] = rotor_velocity_min;
            else
                rotor_velocity_temp[i] = (sqrt(desired_allocator_vector[i]/cf) - 0) / (1900 - 0) * (2000 - 0);

            if(rotor_velocity_temp[i] > rotor_velocity_max)
                rotor_velocity_temp[i] = rotor_velocity_max;
        }
        else
        {
            if(desired_allocator_vector[i] < rotor_velocity_min)
                rotor_velocity_temp[i] = rotor_velocity_min;
            else
                rotor_velocity_temp[i] = (sqrt(desired_allocator_vector[i]/ct) - 0) / (16000 - 0) * (2000 - 0);

            if(rotor_velocity_temp[i] > rotor_velocity_max)
                rotor_velocity_temp[i] = rotor_velocity_max;
        }
    }
    *rotors_velocity << rotor_velocity_temp[0], rotor_velocity_temp[1], rotor_velocity_temp[2],
                        rotor_velocity_temp[3], rotor_velocity_temp[4], rotor_velocity_temp[5];

    // *rotors_velocity << 570.00, 590.00, 570.00, 590.00, 570.00, 590.00;

    // ROS_INFO("ControlAllocator calculate the desired rotors velocity: [%f, %f, %f, %f, %f, %f]."
	// 	, (*rotors_velocity)[0]
	// 	, (*rotors_velocity)[1]
	// 	, (*rotors_velocity)[2]
	// 	, (*rotors_velocity)[3]
	// 	, (*rotors_velocity)[4]
	// 	, (*rotors_velocity)[5]);
}