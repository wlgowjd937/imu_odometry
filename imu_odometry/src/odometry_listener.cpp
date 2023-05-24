#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/tracking/kalman_filters.hpp>

// Global variables to store the initial position and orientation
geometry_msgs::Vector3 initial_position;
double initial_yaw = 0.0;

// Global variables to store the previous pose and displacement
geometry_msgs::Vector3 previous_position;
double previous_yaw = 0.0;
ros::Time previous_time;

// Kalman filter variables
KalmanFilter kalman_filter;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // Check if the initial position and orientation have been set
  if (initial_position.x == 0.0 && initial_position.y == 0.0 && initial_position.z == 0.0)
  {
    initial_position = msg->linear_acceleration;
    initial_yaw = 0.0; // Replace with actual calculation if available
    // Initialize the Kalman filter with initial position and orientation
    kalman_filter.init(initial_position.x, initial_position.y, initial_position.z, initial_yaw);
  }

  // Get the current linear acceleration from the IMU data
  double linear_acceleration_x = msg->linear_acceleration.x;
  double linear_acceleration_y = msg->linear_acceleration.y;
  double linear_acceleration_z = msg->linear_acceleration.z;

  // Get the current angular velocity from the IMU data
  double angular_velocity_x = msg->angular_velocity.x;
  double angular_velocity_y = msg->angular_velocity.y;
  double angular_velocity_z = msg->angular_velocity.z;

  // Calculate the time difference since the last measurement
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - previous_time).toSec();

  // Perform Kalman filter prediction step
  kalman_filter.predict(dt);

  // Perform Kalman filter update step using linear acceleration and angular velocity
  kalman_filter.update(linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
                       angular_velocity_x, angular_velocity_y, angular_velocity_z);

  // Get the current estimated position and orientation from the Kalman filter
  double estimated_x, estimated_y, estimated_z, estimated_yaw;
  kalman_filter.getPosition(estimated_x, estimated_y, estimated_z);
  kalman_filter.getOrientation(estimated_yaw);

  // Update the previous pose and displacement
  previous_position.x = estimated_x;
  previous_position.y = estimated_y;
  previous_position.z = estimated_z;
  previous_yaw = estimated_yaw;
  previous_time = current_time;

  // Print the current estimated position
  ROS_INFO_STREAM("Estimated Position: (" << estimated_x << ", " << estimated_y << ", " << estimated_z << ")");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle nh;

  initial_position = geometry_msgs::Vector3();
  previous_position = geometry_msgs::Vector3();
  previous_time = ros::Time::now();

  // Initialize the Kalman filter with appropriate dimensions
  kalman_filter.init(3, 1, 3, 1);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("/camera/imu", 10, imuCallback);

  ros::spin();

  return 0;
}