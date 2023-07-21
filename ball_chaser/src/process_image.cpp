#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

class ProcessImage {
public:
  ProcessImage() {
    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>(
        "ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside
    // the process_image_callback function
    sub1 = n.subscribe("/camera/rgb/image_raw", 10,
                       &ProcessImage::process_image_callback, this);
  }

  void drive_robot(float linear_x, float angular_z) {
    ROS_INFO("Moving the robot towards the ball");

    // Send a request
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    // Call the command_robot service
    if (!client.call(srv)) {
      ROS_ERROR("Failed to call service command_robot");
    }
  }

  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image::ConstPtr &msg) {
    int white_pixel = 255;
    int column_index = 0;

    for (int i = 0; i < msg->height * msg->step; i += 3) {
      int r = msg->data[i];
      int g = msg->data[i + 1];
      int b = msg->data[i + 2];

      if (r == white_pixel && g == white_pixel && b == white_pixel) {
        column_index = i % msg->step;

        if (column_index < msg->step / 3)
          drive_robot(0.5, 1);
        else if (column_index < 2 * msg->step / 3)
          drive_robot(0.5, 0);
        else
          drive_robot(0.5, -1);

        return; // Exit the function once the ball is found
      }
    }

    drive_robot(0, 0); // Ball not found, stop the robot
  }

private:
  ros::ServiceClient client;
  ros::NodeHandle n;
  ros::Subscriber sub1;
};

int main(int argc, char **argv) {
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");

  ProcessImage node;

  // Handle ROS communication events
  ros::spin();

  return 0;
}