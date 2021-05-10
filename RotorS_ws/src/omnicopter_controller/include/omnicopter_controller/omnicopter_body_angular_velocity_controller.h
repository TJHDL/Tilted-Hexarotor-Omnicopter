#include "stdio.h"
#include "ros/ros.h"
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include "omnicopter_controller/common.h"
#include "omnicopter_controller/parameters.h"

//Default body inertial
#define Jxx 2.420e-01
#define Jyy 2.420e-01
#define Jzz 4.627e-01
#define delta_T 0.1    //constant time value

//Global variable
mav_msgs::EigenRateThrust body_angular_velocity_;
mav_msgs::EigenOdometry angular_velocity_odometry_;
Eigen::Vector3d desired_body_total_torque;
Eigen::Matrix3d body_inertia_matrix;

void SetAngularVelocity(mav_msgs::EigenRateThrust& angular_velocity1) 
{
    body_angular_velocity_ = angular_velocity1;
}

void SetAngularVelocityOdometry(mav_msgs::EigenOdometry& odometry1) 
{
    angular_velocity_odometry_ = odometry1;
}

void CalculateDesiredTotalTorque(Eigen::Vector3d* desired_body_torque)
{
    body_inertia_matrix << Jxx, 0, 0,
                            0, Jyy, 0,
                            0, 0, Jzz;

    Eigen::Vector3d differential_body_angular_velocity;
    differential_body_angular_velocity = (body_angular_velocity_.angular_rates 
        - angular_velocity_odometry_.angular_velocity_B) / delta_T;

    Eigen::Vector3d Body_Inertia_Matrix_Dot_Angular_Velocity;
    Body_Inertia_Matrix_Dot_Angular_Velocity = body_inertia_matrix * angular_velocity_odometry_.angular_velocity_B;
    
    Eigen::MatrixXd Angular_Velocity_Moment_Cross_Matrix(3,1);
    Angular_Velocity_Moment_Cross_Matrix = angular_velocity_odometry_.angular_velocity_B.cross(Body_Inertia_Matrix_Dot_Angular_Velocity);

    Eigen::MatrixXd result(3,1);
    result = body_inertia_matrix * differential_body_angular_velocity + Angular_Velocity_Moment_Cross_Matrix;

    *desired_body_torque << result(0, 0), result(1, 0), result(2, 0);

    // ROS_INFO("BodyAngularVelocityController calculated the desired body torque: [%f, %f, %f]."
	// 	, body_angular_velocity_.angular_rates.x()
	// 	, body_angular_velocity_.angular_rates.y()
	// 	, body_angular_velocity_.angular_rates.z());
}