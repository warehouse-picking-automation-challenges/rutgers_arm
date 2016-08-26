/*
 * ros_caffe_test.cpp
 *
 *  Created on: Aug 31, 2015
 *      Author: Tzutalin
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>

#include "prx/network_t.hpp"
#include "prx/utilities/spaces/space.hpp"


const std::string RECEIVE_IMG_TOPIC_NAME = "/toy_car_input";
const std::string PUBLISH_RET_TOPIC_NAME = "/toy_car_output";

typedef std::vector<float> Prediction;

network_t<std::vector<float> >* toy_car_mlp;
std::string model_path;
std::string weights_path;
std::string mean_file;
std::string min_file;
std::string max_file;

prx::util::space_point_t * space_point;

ros::Publisher gPublisher;

void publishRet(Prediction& prediction);

void inputCallback(const std_msgs::Float32MultiArray::ConstPtr& input) {
    float data[8];
    for (int i = 0; i < (sizeof(data) / sizeof(data[0])); i++) {
        data[i] = input->data[i];
    }

    std::vector<float> input_data(data, data + (sizeof(data) / sizeof(data[0])));

    Prediction prediction = toy_car_mlp->forward_pass(input_data);
	
    publishRet(prediction);
}

// TODO: Define a msg or create a service
// Try to receive : $rostopic echo /caffe_ret
void publishRet(Prediction& prediction)  {
    std_msgs::Float32MultiArray msg;
    for (int i = 0; i < prediction.size(); i++) {
      msg.data.push_back(prediction[i]);
    }
    gPublisher.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_caffe_test");
    ros::NodeHandle nh;
    // To receive an image from the topic, PUBLISH_RET_TOPIC_NAME
    ros::Subscriber sub = nh.subscribe(RECEIVE_IMG_TOPIC_NAME, 1, inputCallback);
	gPublisher = nh.advertise<std_msgs::Float32MultiArray>(PUBLISH_RET_TOPIC_NAME, 100);
    const std::string ROOT_SAMPLE = ros::package::getPath("prx_learn");
    model_path = ROOT_SAMPLE + "/models/toy_car/toycar_2fc_deploy.prototxt";
    weights_path = ROOT_SAMPLE + "/models/toy_car/2fc_iter_20001.caffemodel";
    mean_file = ROOT_SAMPLE + "/data/toy_car/toy_car_mean.binaryproto";
    min_file = ROOT_SAMPLE + "/data/toy_car/toy_car_min.binaryproto";
    max_file = ROOT_SAMPLE + "/data/toy_car/toy_car_max.binaryproto";

    toy_car_mlp = new network_t<std::vector<float> >(model_path, weights_path, mean_file, min_file, max_file);

    ros::spin();
    delete toy_car_mlp;
    ros::shutdown();
    return 0;
}
