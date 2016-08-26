#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <std_msgs/Float32MultiArray.h>

std::vector<float> get_message(std::ifstream & data_in) {

  std::string in_state, in_action, in_duration, in;

  std::getline(data_in, in_state);
  std::getline(data_in, in_action);
  if (in_action.length() < 2)
    return get_message(data_in);
  std::getline(data_in, in_duration);

  in = in_state + ',' + in_action + ',' + in_duration;

  char * writable = new char[in.size() + 1];
  std::copy(in.begin(), in.end(), writable);
  writable[in.size()] = '\0';

  // Now vectorize the string

  std::vector<float> out_array;
  char * word; 
  word = strtok (writable, ", ");
  while (word != NULL) {
    out_array.push_back(atof(word));

    word = strtok(NULL, ", ");
  }

  return out_array;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "data_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("toy_car_input", 1000);
  ros::Rate loop_rate(10);

  int count = 0;

  std::ifstream data_in;
  data_in.open("/home/colin/Repos/fast-rcnn/caffe-fast-rcnn/examples/toy_car/data_output_50Hz.txt");

  while (ros::ok() && data_in.is_open())
  {
    // Continuously read and publish lines of data from input

    std_msgs::Float32MultiArray msg;

    std::vector<float> in_vec = get_message(data_in);

    for (int i = 0; i < in_vec.size(); i++) {
      msg.data.push_back(in_vec[i]);
    }

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  data_in.close();

  return 0;
}

