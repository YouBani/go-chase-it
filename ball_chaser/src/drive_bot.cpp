#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

class DriveBot {
public:
  DriveBot() {
    // Create a publisher
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Create a service server for command_robot request
    service_ = n.advertiseService("/ball_chaser/command_robot",
                                  &DriveBot::handle_drive_request, this);
    ROS_INFO("Ready to send drive commands");
  }

  bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
                            ball_chaser::DriveToTarget::Response &res) {
    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f",
             (float)req.linear_x, (float)req.angular_z);

    // Pusblish motor command request
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    motor_command_publisher.publish(motor_command);

    // Wait for 3 seconds for robot to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback =
        "Motor command set - linear_x: " + std::to_string(req.linear_x) +
        " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

private:
  ros::NodeHandle n;
  ros::Publisher motor_command_publisher;
  ros::ServiceServer service_;
};

int main(int argc, char **argv) {
  // Initialize the drive_bot node and create a handle to it
  ros::init(argc, argv, "drive_bot");

  // Create an object of class DriveBot that will take care of everything
  DriveBot driveBot;

  // Handle ROS communication events
  ros::spin();

  return 0;
}
