/**
 * @file main.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */
#ifdef SIMTRACK_GH_FOUND

#include <apc_shelf.hpp>

#include <stdlib.h>
#include <boost/function.hpp>
#include <deque>
#include <algorithm>
#include <iterator>
#include <string>

#include <fstream>
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <ros/callback_queue.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include "ros/ros.h"
#include "prx_sensing/UpdateObjectList.h"
#include "prx_sensing/PublishObjectList.h"
#include "prx_sensing/UpdateShelfPosition.h"
#include "prx_sensing/SwitchCameras.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <boost/assign/list_of.hpp>

#include "simtrack_nodes/SwitchObjects.h"
#include "simtrack_nodes/poseMsg.h"

#include "objectDetectCNN_pkg/TurnOn.h"
#include "objectDetectCNN_pkg/BoundingBox.h"
#include "objectDetectCNN_pkg/UpdateActive.h"

std::vector<std::string> class_names = boost::assign::list_of("barkely_hide_bones")("command_hooks")("cool_shot_glue_sticks")("creativity_chenille_stems_1")("creativity_chenille_stems_2")("dove_beauty_bar")("dr_browns_bottle_brush")("elmers_washable_no_run_school_glue")("expo_dry_erase_board_eraser")("folgers_classic_roast_coffee")("hanes_tube_socks")("i_am_a_bunny_book")("jane_eyre_dvd")("kleenex_tissue_box")("oral_b_toothbrush_green")("oral_b_toothbrush_red")("laugh_out_loud_joke_book")("peva_shower_curtain_liner")("platinum_pets_dog_bowl")("rolodex_jumbo_pencil_cup")("scotch_bubble_mailer")("safety_first_outlet_plugs")("scotch_duct_tape")("ticonderoga_12_pencils")("up_glucose_bottle")("soft_white_lightbulb")("staples_index_cards")("crayola_24_ct")("kleenex_paper_towels")("dasani_water_bottle")("rawlings_baseball")("fitness_gear_3lb_dumbbell")
("woods_extension_cord")("easter_turtle_sippy_cup")("cherokee_easy_tee_shirt")("fiskars_scissors_red")("cloud_b_plush_bear")("clorox_utility_brush")("kyjen_squeakin_eggs_plush_puppies")("womens_knit_gloves");

typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyRGBD;
typedef message_filters::Synchronizer<SyncPolicyRGBD> SynchronizerRGBD;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyRGB;
typedef message_filters::Synchronizer<SyncPolicyRGB> SynchronizerRGB;

boost::shared_ptr<SynchronizerRGB> sync_rgb_;
boost::shared_ptr<SynchronizerRGBD> sync_rgbd_;

boost::shared_ptr<image_transport::ImageTransport> kinect_image_it, kinect_depth_it;
image_transport::SubscriberFilter kinect_image_sub, kinect_depth_sub;

ros::NodeHandle* PtrNodeHandle;
std::vector<std::string> object_names;
std::vector<std::string> active_objects;
std::map<std::string, simtrack_nodes::poseMsg> active_poses;
std::string simtrack_main_node_name;
std::string simtrack_head_node_name;
std::string tf_namespace;
std::string node_name;
std::map<std::string, ros::Publisher> pose_pubs;
apc_shelf* shelf;

int bin_id = 1;
tf::Vector3 top_left, top_right, bottom_left, bottom_right;

boost::shared_ptr<image_transport::ImageTransport> main_image_it;
image_transport::Publisher main_image_pub;

boost::shared_ptr<image_transport::ImageTransport> main_depth_it;
image_transport::Publisher main_depth_pub;

ros::Publisher main_image_info;

std::string robot_camera_frame_id;
bool color_only_mode;

message_filters::Subscriber<sensor_msgs::CameraInfo>* main_camera_info;

std::ofstream log_file;

// TF stuff for transforming poses to world frame before publishing
tf::TransformListener * world_tf_listener;

std::map<std::string, tf::Vector3> shelf_tfs;
std::vector<std::string> shelf_tf_names = boost::assign::list_of("left_side")("left_divider") 
        ("right_divider")( "right_side")( "top_shelf")( "middle_shelf")( "bottom_shelf");


/*
    Update the list of desired object poses maintained by this node 
    ServiceCall returns boolean indicating successful update
*/

namespace cv
{

    template <typename T>
    void balanceWhite(std::vector < Mat_<T> > &src, Mat &dst,
        const float inputMin, const float inputMax,
        const float outputMin, const float outputMax, const int algorithmType)
    {
        switch ( algorithmType )
        {
            case 1:
                {
                    /********************* Simple white balance *********************/
                    float s1 = 5.0f; // low quantile
                    float s2 = 0.0f; // high quantile

                    int depth = 2; // depth of histogram tree
                    if (src[0].depth() != CV_8U)
                        ++depth;
                    int bins = 16; // number of bins at each histogram level

                    int nElements = int( pow((float)bins, (float)depth) );
                     // number of elements in histogram tree

                    for (size_t i = 0; i < src.size(); ++i)
                    {
                        std::vector <int> hist(nElements, 0);

                        typename Mat_<T>::iterator beginIt = src[i].begin();
                        typename Mat_<T>::iterator endIt = src[i].end();

                        for (typename Mat_<T>::iterator it = beginIt; it != endIt; ++it)
                         // histogram filling
                        {
                            int pos = 0;
                            float minValue = inputMin - 0.5f;
                            float maxValue = inputMax + 0.5f;
                            T val = *it;

                            float interval = float(maxValue - minValue) / bins;

                            for (int j = 0; j < depth; ++j)
                            {
                                int currentBin = int( (val - minValue + 1e-4f) / interval );
                                if (val != 0)
                                    ++hist[pos + currentBin];

                                pos = (pos + currentBin)*bins;

                                minValue = minValue + currentBin*interval;
                                maxValue = minValue + interval;

                                interval /= bins;
                            }
                        }

                        int total = int( src[i].total() );

                        int p1 = 0, p2 = bins - 1;
                        int n1 = 0, n2 = total;

                        float minValue = inputMin - 0.5f;
                        float maxValue = inputMax + 0.5f;

                        float interval = (maxValue - minValue) / float(bins);

                        for (int j = 0; j < depth; ++j)
                         // searching for s1 and s2
                        {
                            while (n1 + hist[p1] < s1 * total / 100.0f)
                            {
                                n1 += hist[p1++];
                                minValue += interval;
                            }
                            p1 *= bins;

                            while (n2 - hist[p2] > (100.0f - s2) * total / 100.0f)
                            {
                                n2 -= hist[p2--];
                                maxValue -= interval;
                            }
                            p2 = p2*bins - 1;

                            interval /= bins;
                        }

                        src[i] = (outputMax - outputMin) * (src[i] - minValue)
                            / (maxValue - minValue) + outputMin;
                    }
                    /****************************************************************/
                    break;
                }
            default:
                break;
        }

        dst.create(/**/ src[0].size(), CV_MAKETYPE( src[0].depth(), int( src.size() ) ) /**/);
        cv::merge(src, dst);
    }

    /*!
    * Wrappers over different white balance algorithm
    *
    * \param src : source image (RGB)
    * \param dst : destination image
    *
    * \param inputMin : minimum input value
    * \param inputMax : maximum input value
    * \param outputMin : minimum output value
    * \param outputMax : maximum output value
    *
    * \param algorithmType : type of the algorithm to use
    */
    void balanceWhite(const Mat &src, Mat &dst, const int algorithmType,
        const float inputMin, const float inputMax,
        const float outputMin, const float outputMax)
    {
        switch ( src.depth() )
        {
            case CV_8U:
                {
                    std::vector < Mat_<uchar> > mv;
                    split(src, mv);
                    balanceWhite(mv, dst, inputMin, inputMax, outputMin, outputMax, algorithmType);
                    break;
                }
            case CV_16S:
                {
                    std::vector < Mat_<short> > mv;
                    split(src, mv);
                    balanceWhite(mv, dst, inputMin, inputMax, outputMin, outputMax, algorithmType);
                    break;
                }
            case CV_32S:
                {
                    std::vector < Mat_<int> > mv;
                    split(src, mv);
                    balanceWhite(mv, dst, inputMin, inputMax, outputMin, outputMax, algorithmType);
                    break;
                }
            case CV_32F:
                {
                    std::vector < Mat_<float> > mv;
                    split(src, mv);
                    balanceWhite(mv, dst, inputMin, inputMax, outputMin, outputMax, algorithmType);
                    break;
                }
            default:
                CV_Error_( CV_StsNotImplemented,
                    ("Unsupported source image format (=%d)", src.type()) );
                break;
        }
    }
}


bool get_shelf_tfs () {

    tf::StampedTransform shelf_tf;
    std::string from_frame = "map";

    for (std::vector<std::string>::size_type i = 0; i != shelf_tf_names.size(); i++ ) {
        std::string name = shelf_tf_names[i];
        std::string to_frame = "simulation/simulator/obstacles/shelf/" + name;
        try {
            world_tf_listener->lookupTransform(from_frame, to_frame, ros::Time(0), shelf_tf);
        }
        catch (tf::TransformException except) {
            PRX_WARN_S("prx_sensing found no TF from "
                        << from_frame.c_str() << " to " << to_frame.c_str()
                        << " -> " << except.what());
        }

        tf::Vector3 tf_location = shelf_tf.getOrigin();
        shelf_tfs[name] = tf_location;
    }

    return true;
}

void parameterError(std::string function_name,
                                      std::string topic_name) {
  std::stringstream err;
  err << "MultiRigidNode::" << function_name << ": could not find "
      << topic_name << " on parameter server" << std::endl;
  throw std::runtime_error(err.str());
}

bool update_object_list(prx_sensing::UpdateObjectList::Request  &req,
         prx_sensing::UpdateObjectList::Response &res) {

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<
            simtrack_nodes::SwitchObjects>(simtrack_main_node_name + "/switch_objects");
    ros::ServiceClient client_cnn = n.serviceClient<
            objectDetectCNN_pkg::UpdateActive>( "update_active_list");
    ros::ServiceClient client_cnn_on = n.serviceClient<
            objectDetectCNN_pkg::TurnOn>( "rcnn/turn_on");

    get_shelf_tfs();
    if ((req.bin_id >= 1) && (req.bin_id <= 12))
        bin_id = req.bin_id;

    simtrack_nodes::SwitchObjects srv;
    int obj_list_sz = sizeof(req.object_list)/sizeof(req.object_list[0]);
    std::vector<std::string> new_object_list = req.object_list;
    for (int i = 0; i < new_object_list.size(); i++){
        if (std::count(new_object_list.begin(), 
                        new_object_list.end(), 
                        new_object_list[i]) > 1)
            new_object_list[i] = new_object_list[i] + "_DUP";
        // else
        //     new_object_list.push_back(req.object_list[i]);
    }

    srv.request.model_names = new_object_list;
    active_objects = new_object_list;

    if (client.call(srv))
    {
        ROS_INFO("Called service simtrack/SwitchObjects\n");
        // ROS_INFO("Object List Now: %s", req.object_list);
    }
    else
    {
        ROS_ERROR("Failed to call service simtrack/SwitchObjects");
        return 1;
    }

    // Switch CNN object list
    objectDetectCNN_pkg::UpdateActive srv_cnn;
    // std::vector<int> cnn_object_list;
    for (int current = 0; current < req.object_list.size(); current++){
        for (int found = 0; found < class_names.size(); found++) {
            if (class_names[found] == req.object_list[current])
                srv_cnn.request.active_list.push_back(found);
        }
    }

    // srv.request.model_names = new_object_list;
    // active_objects = new_object_list;

    if (client_cnn.call(srv_cnn))
    {
        ROS_INFO("Called service rcnn/update_active_list\n");
    }
    else
    {
        ROS_ERROR("Failed to call service rcnn/update_active_list");
        return 1;
    }

    // Turn CNN detection ON
    objectDetectCNN_pkg::TurnOn srv_cnn_on;
    srv_cnn_on.request.input = true;

    if (client_cnn_on.call(srv_cnn_on))
    {
        ROS_INFO("Called service rcnn/turn_on\n");
    }
    else
    {
        ROS_ERROR("Failed to call service rcnn/turn_on");
        return 1;
    }

    res.response = true;
}

/* 
    Check all current pose estimates against the active object list
    ServiceCall Returns only those which have been requested     
*/
bool publish_object_list(prx_sensing::PublishObjectList::Request  &req,
         prx_sensing::PublishObjectList::Response &res) {

    ros::NodeHandle n;

    std::vector<prx_sensing::ObjectPose> ret_poses;

    std::cout << "Active object list: ";
    for (int j = 0; j < active_objects.size(); j++) {
        std::cout << active_objects[j] << ", ";
    }
    std::cout << std::endl;
    
    log_file.open("detection_log.txt", ios::out | ios::app);
    std::string out_str;

    for (int i = 0; i < active_objects.size(); i++) {
        geometry_msgs::PoseStamped found = active_poses[active_objects[i]].pose;

        if(found.pose.orientation.x != found.pose.orientation.x){
            // out_str = "Not Found: " +  active_objects[i];
            // log_file << out_str << "\n";
            continue;
        }
        else if ((found.pose.orientation.x == 0.0) &&
            (found.pose.orientation.y == 0.0) &&
            (found.pose.orientation.z == 0.0)){
            // out_str = "Not Found: " +  active_objects[i];
            // log_file << out_str << "\n";
            continue; 
        }
        else {


            PRX_INFO_S("Publishing " << active_objects[i] << ": ("
                        << found.pose.position << "), ("
                        << found.pose.orientation << ")");

            // (2) Publish the world frame transform to simulation
            prx_sensing::ObjectPose new_obj;
            std::string obj_name = active_objects[i];
            new_obj.object_name = obj_name;
            new_obj.pose = found;
            ret_poses.push_back(new_obj);
            log_file << "Found: " <<  active_objects[i] 
                        << "\nR: " << found.pose.orientation.x << " "
                                    << found.pose.orientation.y << " "
                                    << found.pose.orientation.z << " "
                                    << found.pose.orientation.w
                        << "\nT: " << found.pose.position.x << " "
                                    << found.pose.position.y << " "
                                    << found.pose.position.z << "\n";;        
        }
    }
    
    for (int j = 0; j < ret_poses.size(); j++ ) {
        std::cout << "Publishing: " << ret_poses[j].object_name << std::endl;
    }

    res.object_list = ret_poses;

    active_poses.clear();
    log_file.close();

    // Turn CNN detection OFF
    ros::ServiceClient client_cnn_on = n.serviceClient<
        objectDetectCNN_pkg::TurnOn>( "rcnn/turn_on");

    objectDetectCNN_pkg::TurnOn srv_cnn_on;
    srv_cnn_on.request.input = false;

    if (client_cnn_on.call(srv_cnn_on))
    {
        ROS_INFO("Called service rcnn/turn_on\n");
    }
    else
    {
        ROS_ERROR("Failed to call service rcnn/turn_on");
        return 1;
    }

    return true;
}

/*
Update Shelf Pose
*/
bool update_shelf_pose(prx_sensing::UpdateShelfPosition::Request  &req,
         prx_sensing::UpdateShelfPosition::Response &res) {

    shelf = new apc_shelf();
   
    if(get_transform("base_link", "kinect2_head_rgb_optical_frame", shelf->CameraToWorldTf) == 0)
    {
        PRX_WARN_S("Shelf Calibration :: Couldn't find TF !!! Returning Default pose.");
        res.shelf_pose = shelf->getDefaultShelfPose();
        return true;
    }

    int good_estimates = 0;
    int time_spent = 0;

    while(1)
    {
        time_spent++;
        if(time_spent > 5){
            PRX_WARN_S("Shelf Calibration Timeout :: Returning Default Pose");
            res.shelf_pose = shelf->getDefaultShelfPose();
            break;
        }

        PointCloud::Ptr current_cloud(new PointCloud);

        sensor_msgs::PointCloud2::ConstPtr msg = 
            ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect2_head/qhd/points", *PtrNodeHandle, ros::Duration(6.0));

        if(msg == NULL)
        {
            PRX_WARN_S("Shelf Calibration :: No Point Cloud");
            continue;
        }

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*current_cloud);

        // if(current_cloud->points.size() == 0)
        // {
        //     PRX_WARN_S("Shelf Calibration :: No Point Cloud");
        //     continue;
        // }

        if(shelf->GetLeftRightPoses(current_cloud) == 0){
            PRX_WARN_S("Shelf Calibration :: Shelf Calibration Failure, Can't See Anything ");
            continue;
        }
        if(shelf->smoothenShelfEstimate(res.shelf_pose) == 0)
            PRX_WARN_S("Shelf Calibration :: Estimate Rejected, Out of Bounds");
        else
            good_estimates++;

        if(good_estimates > 2){
            PRX_WARN_S("Shelf Calibration :: SUCCESS !!!! Returning Pose with high Probability");
            break;
        }

        applyTF(current_cloud, 
               tf::Matrix3x3(shelf->CameraToWorldTf.getRotation()),
                   Eigen::Vector3d(shelf->CameraToWorldTf.getOrigin()));

        visualize_point_cloud(current_cloud);
    }

    shelf->defShelfPose = res.shelf_pose;
    PrintPose(res.shelf_pose, "Final Pose");
    PrintPose(shelf->defShelfPose, "Default Pose");
    shelf->init = 1;
    return true;
}

/*
    Subscriber Callback function for all poses published by Simtrack
    Current: Just passes through all poses to our internal storage
    Future: This is where we should post-process detections
*/
void simtrack_main_pose_cb(const simtrack_nodes::poseMsg::ConstPtr &msg, const std::string &object) {
    
    simtrack_nodes::poseMsg pose = *msg;

    tf::StampedTransform world_tf;

    std::string from_frame   = "base_link";
    std::string to_frame;
    ros::param::get(node_name + "/current_camera_frame", to_frame);

    try {
        world_tf_listener->lookupTransform(from_frame, to_frame, 
                                            ros::Time(0), world_tf);
    }
    catch (tf::TransformException except) {
        PRX_WARN_S("prx_sensing found no TF from "
                    << from_frame.c_str() << " to " << to_frame.c_str()
                    << " -> " << except.what());
    }

    if(pose.pose.pose.orientation.x != pose.pose.pose.orientation.x)
        return;
    else {
        // After finding a matching object to update from prx_sensing node,
        // (1) Transform rotation based on camera rotation (currently head cam)
        tf::Quaternion obj_rot(pose.pose.pose.orientation.x,
                                    pose.pose.pose.orientation.y,
                                    pose.pose.pose.orientation.z,
                                    pose.pose.pose.orientation.w);
        tf::Vector3 obj_trans(pose.pose.pose.position.x,
                                pose.pose.pose.position.y,
                                pose.pose.pose.position.z);
        tf::Transform obj_tf(obj_rot, obj_trans);

        obj_tf = world_tf * obj_tf;
        obj_trans = obj_tf.getOrigin();
        obj_rot = obj_tf.getRotation();

        // found.setOrigin(obj_trans);
        // found.setRotation(obj_rot);
        pose.pose.pose.position.x = obj_trans.getX();
        pose.pose.pose.position.y = obj_trans.getY();
        pose.pose.pose.position.z = obj_trans.getZ();
        pose.pose.pose.orientation.x = obj_rot[0];
        pose.pose.pose.orientation.y = obj_rot[1];
        pose.pose.pose.orientation.z = obj_rot[2];
        pose.pose.pose.orientation.w = obj_rot.getW();
    }

    if(pose.pose.pose.orientation.x != pose.pose.pose.orientation.x)
        return;
    if ((pose.pose.pose.orientation.x == 0.0) &&
        (pose.pose.pose.orientation.y == 0.0) &&
        (pose.pose.pose.orientation.z == 0.0) 
        // && (pose.pose.orientation.w == 1.0)
        )
        return;
    std::cout << "\nObj: " << object << "\nReliability: " << pose.reliability;
    std::cout << "\nOld Reliability: " << active_poses[object].reliability; 
    if ((pose.inliers >= active_poses[object].inliers) && (pose.reliability > active_poses[object].reliability)) {
        
        std::cout << "\nACCEPTED";
        active_poses[object] = pose;
    }
    else
        std::cout << "\nDENIED";
}

void simtrack_head_pose_cb(const simtrack_nodes::poseMsg::ConstPtr &msg, const std::string &object) {
    
    simtrack_nodes::poseMsg pose = *msg;
    if (active_poses.count(object) > 0)
        return;
    else {
        simtrack_nodes::poseMsg pose = *msg;

        tf::StampedTransform world_tf;

        std::string from_frame   = "base_link";
        std::string to_frame = "kinect2_head_rgb_optical_frame";
        // ros::param::get(, to_frame);

        try {
            world_tf_listener->lookupTransform(from_frame, to_frame, 
                                                ros::Time(0), world_tf);
        }
        catch (tf::TransformException except) {
            PRX_WARN_S("prx_sensing found no TF from "
                        << from_frame.c_str() << " to " << to_frame.c_str()
                        << " -> " << except.what());
        }

        if(pose.pose.pose.orientation.x != pose.pose.pose.orientation.x)
            return;
        else {
            // After finding a matching object to update from prx_sensing node,
            // (1) Transform rotation based on camera rotation (currently head cam)
            tf::Quaternion obj_rot(pose.pose.pose.orientation.x,
                                        pose.pose.pose.orientation.y,
                                        pose.pose.pose.orientation.z,
                                        pose.pose.pose.orientation.w);
            tf::Vector3 obj_trans(pose.pose.pose.position.x,
                                    pose.pose.pose.position.y,
                                    pose.pose.pose.position.z);
            tf::Transform obj_tf(obj_rot, obj_trans);

            obj_tf = world_tf * obj_tf;
            obj_trans = obj_tf.getOrigin();
            obj_rot = obj_tf.getRotation();

            // found.setOrigin(obj_trans);
            // found.setRotation(obj_rot);
            pose.pose.pose.position.x = obj_trans.getX();
            pose.pose.pose.position.y = obj_trans.getY();
            pose.pose.pose.position.z = obj_trans.getZ();
            pose.pose.pose.orientation.x = obj_rot[0];
            pose.pose.pose.orientation.y = obj_rot[1];
            pose.pose.pose.orientation.z = obj_rot[2];
            pose.pose.pose.orientation.w = obj_rot.getW();
        }

        if(pose.pose.pose.orientation.x != pose.pose.pose.orientation.x)
            return;
        if ((pose.pose.pose.orientation.x == 0.0) &&
            (pose.pose.pose.orientation.y == 0.0) &&
            (pose.pose.pose.orientation.z == 0.0) 
            // && (pose.pose.orientation.w == 1.0)
            )
            return;

        if (pose.reliability > active_poses[object].reliability)
            active_poses[object] = pose;
    }
}

std::vector<int> get_mask_corners (std::string to_frame, 
                                    const sensor_msgs::CameraInfoConstPtr &rgb_info_msg) {

    // Find world points for four corners
    switch (bin_id) {
        case 1:
            top_left = tf::Vector3(shelf_tfs["left_side"].getX() - 0.215,
                                    shelf_tfs["left_side"].getY(),
                                    shelf_tfs["top_shelf"].getZ() + 0.30);
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["left_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["top_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());  
            break;                                 
        case 2:
            top_left = tf::Vector3(shelf_tfs["left_divider"].getX() - 0.215,
                                    shelf_tfs["left_divider"].getY(),
                                    shelf_tfs["top_shelf"].getZ() + 0.30);
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["top_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;    
        case 3:
            top_left = tf::Vector3(shelf_tfs["right_divider"].getX() - 0.215,
                                    shelf_tfs["right_divider"].getY(),
                                    shelf_tfs["top_shelf"].getZ() + 0.30);
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_side"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["top_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;            
        case 4:
            top_left = tf::Vector3(shelf_tfs["top_shelf"].getX() - 0.215,
                                    shelf_tfs["left_side"].getY(),
                                    shelf_tfs["top_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["left_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["middle_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;            
        case 5:
            top_left = tf::Vector3(shelf_tfs["top_shelf"].getX() - 0.215,
                                    shelf_tfs["left_divider"].getY(),
                                    shelf_tfs["top_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["middle_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;            
        case 6:
            top_left = tf::Vector3(shelf_tfs["top_shelf"].getX() - 0.215,
                                    shelf_tfs["right_divider"].getY(),
                                    shelf_tfs["top_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_side"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["middle_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;            
        case 7:
            top_left = tf::Vector3(shelf_tfs["middle_shelf"].getX() - 0.215,
                                    shelf_tfs["left_side"].getY(),
                                    shelf_tfs["middle_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["left_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["bottom_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;            
        case 8:
            top_left = tf::Vector3(shelf_tfs["middle_shelf"].getX() - 0.215,
                                    shelf_tfs["left_divider"].getY(),
                                    shelf_tfs["middle_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["bottom_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;                    
        case 9:
            top_left = tf::Vector3(shelf_tfs["middle_shelf"].getX() - 0.215,
                                    shelf_tfs["right_divider"].getY(),
                                    shelf_tfs["middle_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_side"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    shelf_tfs["bottom_shelf"].getZ());            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;                            
        case 10:
            top_left = tf::Vector3(shelf_tfs["bottom_shelf"].getX() - 0.215,
                                    shelf_tfs["left_side"].getY(),
                                    shelf_tfs["bottom_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["left_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    top_left.getZ() - 0.30);            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;                                    
        case 11:
            top_left = tf::Vector3(shelf_tfs["bottom_shelf"].getX() - 0.215,
                                    shelf_tfs["left_divider"].getY(),
                                    shelf_tfs["bottom_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_divider"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    top_left.getZ() - 0.30);            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;                                    
        case 12:
            top_left = tf::Vector3(shelf_tfs["bottom_shelf"].getX() - 0.215,
                                    shelf_tfs["right_divider"].getY(),
                                    shelf_tfs["bottom_shelf"].getZ());
            top_right = tf::Vector3(top_left.getX(),
                                    shelf_tfs["right_side"].getY(),
                                    top_left.getZ());
            bottom_left = tf::Vector3(top_left.getX(),
                                    top_left.getY(),
                                    top_left.getZ() - 0.30);            
            bottom_right = tf::Vector3(bottom_left.getX(),
                                    top_right.getY(),
                                    bottom_left.getZ());    
            break;
        default:
            PRX_WARN_S("Error: bin_id must be an integer value 1-12!!\n");
    }

    // Define points in camera frame
    tf::StampedTransform base_to_cam, point;
    tf::Transform result;

    world_tf_listener->lookupTransform(to_frame, "base_link",
                                        ros::Time(0), base_to_cam);
    point.setIdentity();
    point.setOrigin(top_left);
    result = base_to_cam * point;
    top_left = result.getOrigin();
    
    point.setIdentity();
    point.setOrigin(top_right);
    result = base_to_cam * point;
    top_right = result.getOrigin();

    point.setIdentity();
    point.setOrigin(bottom_left);
    result = base_to_cam * point;
    bottom_left = result.getOrigin();

    point.setIdentity();
    point.setOrigin(bottom_right);
    result = base_to_cam * point;
    bottom_right = result.getOrigin();

    // Project to image plane
    cv::Mat camera_matrix = cv::Mat(3, 4, CV_64F, (void *)rgb_info_msg->P.data()).clone();
    double focal_length_x = camera_matrix.at<double>(0,0);
    double focal_length_y = camera_matrix.at<double>(1,1);
    double center_x = camera_matrix.at<double>(0,2);
    double center_y = camera_matrix.at<double>(1,2);

    int img_tl_x, img_tl_y, img_br_x, img_br_y, img_tr_x, img_tr_y, img_bl_x, img_bl_y;
    img_tl_x = (int) ((top_left.getX() * focal_length_x / top_left.getZ()) + center_x);
    img_tl_y = (int) ((top_left.getY() * focal_length_y / top_left.getZ()) + center_y);
    img_tr_x = (int) ((top_right.getX() * focal_length_x / top_right.getZ()) + center_x);
    img_tr_y = (int) ((top_right.getY() * focal_length_y / top_right.getZ()) + center_y);
    img_br_x = (int) ((bottom_right.getX() * focal_length_x / bottom_right.getZ()) + center_x);
    img_br_y = (int) ((bottom_right.getY() * focal_length_y / bottom_right.getZ()) + center_y);
    img_bl_x = (int) ((bottom_left.getX() * focal_length_x / bottom_left.getZ()) + center_x);
    img_bl_y = (int) ((bottom_left.getY() * focal_length_y / bottom_left.getZ()) + center_y);

    std::vector<int> mask_corners = boost::assign::list_of(img_tl_x)(img_tl_y)
                    (img_tr_x)(img_tr_y)(img_br_x)(img_br_y)(img_bl_x)(img_bl_y);
    return mask_corners;
}

void kinect_images_cb (const sensor_msgs::ImageConstPtr &depth_msg,
    const sensor_msgs::ImageConstPtr &rgb_msg, 
    const sensor_msgs::CameraInfoConstPtr &rgb_info_msg){

    std::vector<int> img_corners = get_mask_corners(robot_camera_frame_id, rgb_info_msg);

    // Mask
    cv_bridge::CvImagePtr cv_rgb_ptr, cv_depth_ptr;
    try {
        if (rgb_msg->encoding == "8UC1") // fix for kinect2 mono output
          cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg);
        else
            cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, "rgb8");
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask = cv::Mat::zeros(cv_rgb_ptr->image.rows, 
                            cv_rgb_ptr->image.cols, CV_16UC1);
    cv::Mat mask_color = cv::Mat::zeros(cv_rgb_ptr->image.rows, 
                            cv_rgb_ptr->image.cols, CV_8UC1);
    
    int img_tl_x, img_tl_y, img_br_x, img_br_y, img_tr_x, img_tr_y, img_bl_x, img_bl_y;
    img_tl_x = std::min(std::max(img_corners[0], 0), cv_rgb_ptr->image.cols);
    img_tl_y = std::min(std::max(img_corners[1], 0), cv_rgb_ptr->image.rows);
    img_tr_x = std::min(std::max(img_corners[2], 0), cv_rgb_ptr->image.cols);
    img_tr_y = std::min(std::max(img_corners[3], 0), cv_rgb_ptr->image.rows);
    img_br_x = std::min(std::max(img_corners[4], 0), cv_rgb_ptr->image.cols);
    img_br_y = std::min(std::max(img_corners[5], 0), cv_rgb_ptr->image.rows);
    img_bl_x = std::min(std::max(img_corners[6], 0), cv_rgb_ptr->image.cols);
    img_bl_y = std::min(std::max(img_corners[7], 0), cv_rgb_ptr->image.rows);    
    
    if ((img_br_x > img_tl_x) && 
        (img_br_y > img_tl_y)){

        vector<cv::Point> ROI_Vertices;
        ROI_Vertices.push_back(cv::Point(img_tl_x, img_tl_y));
        ROI_Vertices.push_back(cv::Point(img_tr_x, img_tr_y));
        ROI_Vertices.push_back(cv::Point(img_br_x, img_br_y));
        ROI_Vertices.push_back(cv::Point(img_bl_x, img_bl_y));

        std::vector<cv::Point> ROI_Poly;
        cv::approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);
         
        // Fill polygon white
        cv::fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 65535, 8, 0); 
        cv::fillConvexPoly(mask_color, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0); 
        
        // mask(Rect(img_tl_x,img_tl_y,img_br_x-img_tl_x,img_br_y-img_tl_y)) = 65535;
        // mask_color(Rect(img_tl_x,img_tl_y,img_br_x-img_tl_x,img_br_y-img_tl_y)) = 255;
    }

    // Cut out ROI and store it in imageDest
    cv::bitwise_and(cv_depth_ptr->image, mask, cv_depth_ptr->image);
    std::vector<cv::Mat> color_channel(3);
    for (int i = 0; i < 3; i++)
    {
        cv::extractChannel(cv_rgb_ptr->image, color_channel[i], i);
        cv::bitwise_and(color_channel[i], mask_color, color_channel[i]);
    }
    cv::merge(color_channel, cv_rgb_ptr->image);

    // vector<Mat> norm_channels;
    // cv::Mat histNorm;

    // cv::cvtColor(cv_rgb_ptr->image, cv_rgb_ptr->image, CV_RGB2YCrCb);
    // cv::split(cv_rgb_ptr->image, norm_channels);
    // cv::equalizeHist(norm_channels[0], norm_channels[0]);
    // cv::equalizeHist(norm_channels[1], norm_channels[1]);

    // cv::equalizeHist(norm_channels[2], norm_channels[2]);

    // cv_rgb_ptr->image = (2.0*cv_rgb_ptr->image)
    //                      - cv::Scalar(30,30, 30);

    // norm_channels[0] = norm_channels[0] + cv::Scalar(75);
    // norm_channels[1] = norm_channels[1] + cv::Scalar(75);
    // norm_channels[2] = norm_channels[2] + cv::Scalar(75);

    // cv::merge(norm_channels, cv_rgb_ptr->image);
    // cv::cvtColor(cv_rgb_ptr->image, cv_rgb_ptr->image, CV_YCrCb2RGB);

    // cv::balanceWhite(cv_rgb_ptr->image, cv_rgb_ptr->image, 1, 0, 255, 0, 255);

    // cv::Mat lab_image, final;
    // cv::cvtColor(cv_rgb_ptr->image, lab_image, CV_BGR2Lab);

    // // Extract the L channel
    // std::vector<cv::Mat> lab_planes(3);
    // cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // // apply the CLAHE algorithm to the L channel
    // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    // clahe->setClipLimit(2);
    // clahe->setTilesGridSize(cv::Size(8,8));
    // cv::Mat dst;
    // clahe->apply(lab_planes[0], dst);

    // // Merge the the color planes back into an Lab image
    // dst.copyTo(lab_planes[0]);
    // cv::merge(lab_planes, lab_image);

    // // convert back to RGB
    // cv::cvtColor(lab_image, lab_image, CV_Lab2BGR);

    // cv::GaussianBlur(lab_image, cv_rgb_ptr->image, cv::Size(0, 0), 5, 5);
    // cv::addWeighted(lab_image, 1.5, cv_rgb_ptr->image, -0.5, 0, cv_rgb_ptr->image);

    // Publish on our publishers
    main_image_pub.publish(cv_rgb_ptr->toImageMsg());
    main_depth_pub.publish(cv_depth_ptr->toImageMsg());
    main_image_info.publish(rgb_info_msg);
}

void rgb_images_cb (const sensor_msgs::ImageConstPtr &rgb_msg, 
    const sensor_msgs::CameraInfoConstPtr &rgb_info_msg){

    std::vector<int> img_corners = get_mask_corners(robot_camera_frame_id, rgb_info_msg);

    // Mask
    cv_bridge::CvImagePtr cv_rgb_ptr;
    try {
        if (rgb_msg->encoding == "8UC1") // fix for kinect2 mono output
            cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg);
        else
            cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, "rgb8");
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask_color = cv::Mat::zeros(cv_rgb_ptr->image.rows, 
                            cv_rgb_ptr->image.cols, CV_8UC1);
    
    int img_tl_x, img_tl_y, img_br_x, img_br_y, img_tr_x, img_tr_y, img_bl_x, img_bl_y;
    img_tl_x = std::min(std::max(img_corners[0], 0), cv_rgb_ptr->image.cols);
    img_tl_y = std::min(std::max(img_corners[1], 0), cv_rgb_ptr->image.rows);
    img_br_x = std::min(std::max(img_corners[2], 0), cv_rgb_ptr->image.cols);
    img_br_y = std::min(std::max(img_corners[3], 0), cv_rgb_ptr->image.rows);
    img_tr_x = std::min(std::max(img_corners[4], 0), cv_rgb_ptr->image.cols);
    img_tr_y = std::min(std::max(img_corners[5], 0), cv_rgb_ptr->image.rows);
    img_bl_x = std::min(std::max(img_corners[6], 0), cv_rgb_ptr->image.cols);
    img_bl_y = std::min(std::max(img_corners[7], 0), cv_rgb_ptr->image.rows);    
    
    if ((img_br_x > img_tl_x) && 
        (img_br_y > img_tl_y)){

        vector<cv::Point> ROI_Vertices;
        ROI_Vertices.push_back(cv::Point(img_tl_x, img_tl_y));
        ROI_Vertices.push_back(cv::Point(img_tr_x, img_tr_y));
        ROI_Vertices.push_back(cv::Point(img_br_x, img_br_y));
        ROI_Vertices.push_back(cv::Point(img_bl_x, img_bl_y));

        std::vector<cv::Point> ROI_Poly;
        cv::approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);
         
        // Fill polygon white
        cv::fillConvexPoly(mask_color, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0); 
    }

    // Cut out ROI and store it in imageDest
    std::vector<cv::Mat> color_channel(3);
    for (int i = 0; i < 3; i++)
    {
        cv::extractChannel(cv_rgb_ptr->image, color_channel[i], i);
        cv::bitwise_and(color_channel[i], mask_color, color_channel[i]);
    }
    cv::merge(color_channel, cv_rgb_ptr->image);

    // Publish on our publishers
    main_image_pub.publish(cv_rgb_ptr->toImageMsg());
}


bool switch_cameras(std::string camera_index) {

  // unsubscribe from all camera topics
  sync_rgbd_.reset();
  sync_rgb_.reset();
  kinect_depth_sub.unsubscribe();
  kinect_depth_it.reset();
  main_camera_info->unsubscribe();
  kinect_image_sub.unsubscribe();
  kinect_image_it.reset();

  bool compressed_streams = false;
  ros::param::get("simtrack/use_compressed_streams", compressed_streams);

  image_transport::TransportHints rgb_hint, depth_hint;
  if (compressed_streams) {
    rgb_hint = image_transport::TransportHints("compressed");
    depth_hint = image_transport::TransportHints("compressedDepth");
  } else {
    rgb_hint = image_transport::TransportHints("raw");
    depth_hint = image_transport::TransportHints("raw");
  }

  // fetch rgb topic names from parameter server
  std::stringstream topic_name;
  topic_name << "/camera/" << camera_index << "/rgb";
  std::string rgb_topic;
  if (!ros::param::get(topic_name.str(), rgb_topic))
    parameterError(__func__, topic_name.str());

  topic_name.str("");
  topic_name << "/camera/" << camera_index << "/rgb_info";
  std::string rgb_info_topic;
  if (!ros::param::get(topic_name.str(), rgb_info_topic))
    parameterError(__func__, topic_name.str());

  kinect_image_it.reset(new image_transport::ImageTransport(*PtrNodeHandle));
  kinect_image_sub.subscribe(*kinect_image_it, rgb_topic, 1, rgb_hint);
  main_camera_info->subscribe(*PtrNodeHandle, rgb_info_topic, 1);

  topic_name.str("");
  topic_name << "/camera/" << camera_index << "/robot_frame";
  if (!ros::param::get(topic_name.str(), robot_camera_frame_id))
    parameterError(__func__, topic_name.str());

  topic_name.str("");
  topic_name << "/camera/" << camera_index << "/color_only_mode";
  if (!ros::param::get(topic_name.str(), color_only_mode))
    parameterError(__func__, topic_name.str());

  if (color_only_mode) {
    sync_rgb_.reset(
        new SynchronizerRGB(SyncPolicyRGB(5), kinect_image_sub, *main_camera_info));
    sync_rgb_->registerCallback(
        boost::bind(&rgb_images_cb, _1, _2));
  } else {
    topic_name.str("");
    topic_name << "/camera/" << camera_index << "/depth";
    std::string depth_topic;
    if (!ros::param::get(topic_name.str(), depth_topic))
      parameterError(__func__, topic_name.str());

    kinect_depth_it.reset(new image_transport::ImageTransport(*PtrNodeHandle));
    kinect_depth_sub.subscribe(*kinect_depth_it, depth_topic, 1, depth_hint);
    sync_rgbd_.reset(new SynchronizerRGBD(SyncPolicyRGBD(5), kinect_depth_sub,
                                          kinect_image_sub, *main_camera_info));
    sync_rgbd_->registerCallback(
        boost::bind(&kinect_images_cb, _1, _2, _3));
  }

  std::cout << "Switched frame to: " << robot_camera_frame_id << std::endl;
  ros::param::set(node_name + "/current_camera_frame", robot_camera_frame_id);
  std::cout << "After update param: " << robot_camera_frame_id << std::endl;

  return true;
}


bool switch_cameras_srv(prx_sensing::SwitchCameras::Request  &req,
         prx_sensing::SwitchCameras::Response &res) {
    
    std::string camera_index = req.camera_index;
    bool ret_val = switch_cameras(camera_index);
    res.ret_val = ret_val;
    return true;
}

int main( int ac, char* av[] ) {

    // ros::init(ac, av, "sensing");
    std::string filename="";
    if (ac == 1)
    {
        node_name = "sensing";
    }
    else
    {
        node_name = av[1];
        // This skips the first 8 characters, which by default in ros is: __name:=
        std::string test = node_name.substr(0,8);
        if (test == "__name:=")
        {
            node_name = node_name.substr(8);
        }
    }

    ros::init(ac, av, node_name);
    ros::NodeHandle main_node_handle;
    PtrNodeHandle = &main_node_handle;
    world_tf_listener = new tf::TransformListener;

    if(ros::param::has("yaml_input"))
    {
        ros::param::get("yaml_input",filename);
    }

    // Wait for parameter setting scripts to finish.
    while (ros::param::has("prx/parameter_mutex")) {}

    // PUBLISHERS    
    // TODO: make this dynamic as to which topic to listen to
    image_transport::TransportHints kinect_hint;
    kinect_hint = image_transport::TransportHints("raw");

    // kinect_image_it.reset(new image_transport::ImageTransport(main_node_handle));
    // kinect_image_sub.subscribe(*kinect_image_it, "kinect2_left/qhd/image_color_rect", 
    //                             2, kinect_hint);
    // kinect_depth_it.reset(new image_transport::ImageTransport(main_node_handle));
    // kinect_depth_sub.subscribe(*kinect_image_it, "kinect2_left/qhd/image_depth_rect", 
    //                             2, kinect_hint);

    main_camera_info = new message_filters::Subscriber<sensor_msgs::CameraInfo>
                                            (main_node_handle, "kinect2_left/qhd/camera_info", 2);

    // sync_rgbd_.reset(new SynchronizerRGBD(SyncPolicyRGBD(5), kinect_depth_sub,
    //                                         kinect_image_sub, *main_camera_info));
    // sync_rgbd_->registerCallback(boost::bind(&kinect_images_cb, _1, _2, _3));

    main_image_it.reset(new image_transport::ImageTransport(main_node_handle));
    main_image_pub = main_image_it->advertise("main_simtrack_image", 1);

    main_depth_it.reset(new image_transport::ImageTransport(main_node_handle));
    main_depth_pub = main_depth_it->advertise("main_simtrack_depth", 1);

    main_image_info = main_node_handle.advertise<sensor_msgs::CameraInfo>("main_simtrack_camera_info", 1);


    ros::ServiceServer update_object_list_service = main_node_handle
                                                    .advertiseService("prx/"+node_name+"/update_object_list", update_object_list);

    ros::ServiceServer publish_object_list_service = main_node_handle
                                                    .advertiseService("prx/"+node_name+"/publish_object_list", publish_object_list);

    ros::ServiceServer shelf_estimation_service = main_node_handle
                                                    .advertiseService("prx/"+node_name+"/update_shelf_pose", update_shelf_pose);
    
    ros::ServiceServer switch_camera_service = main_node_handle
                                                    .advertiseService("prx/"+node_name+"/switch_cameras", switch_cameras_srv);                          


    // Block initialization until after robot TFs have been published
    std::string camera_frame;
    ros::param::get(node_name + "/current_camera_frame", camera_frame);
    world_tf_listener->waitForTransform("base_link", camera_frame, 
                                        ros::Time(0), ros::Duration(1000000));
    world_tf_listener->waitForTransform("map", 
                                        "simulator/consumer/manipulator/left_RGB_camera", 
                                        ros::Time(0), ros::Duration(1000000));

    switch_cameras("left_kinect");

    // Call switch_cameras service with default (left_kinect) as argument
    // ros::ServiceClient switch_cams = main_node_handle.serviceClient<prx_sensing::SwitchCameras>("prx/sensing/switch_cameras");
    // prx_sensing::SwitchCameras srv;
    // srv.request.camera_index = "left_kinect";
    // if (switch_cams.call(srv))
    // {
    //     ROS_INFO("Switching cameras to: left_kinect");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service SwitchCameras");
    //     return 1;
    // }

    ros::param::get(node_name+"/object_names", object_names);
    active_objects = object_names;
    ros::param::get(node_name+"/simtrack_main_node_name", simtrack_main_node_name);
    ros::param::get(node_name+"/simtrack_head_node_name", simtrack_head_node_name);
    std::map<std::string, ros::Subscriber> pose_subs_main;
    std::map<std::string, ros::Subscriber> pose_subs_head;
    std::cout<<simtrack_main_node_name<<"\n";

    ros::param::get(node_name+"/tf_namespace", tf_namespace);
    std::cout << "Using non-default value for tf_namespace: " << tf_namespace << std::endl;

    // SUBSCRIBERS
    for(int i=0;i<object_names.size(); ++i)
    {
        std::cout<<"Object ["<<i<<"] : "<<object_names[i]<<"\n";
        pose_subs_main[object_names[i]] = main_node_handle.subscribe<simtrack_nodes::poseMsg>(
                                     "/"+simtrack_main_node_name+"/"+object_names[i],
                                     100,
                                     boost::bind(simtrack_main_pose_cb, _1, object_names[i]));

        pose_subs_head[object_names[i]] = main_node_handle.subscribe<simtrack_nodes::poseMsg>(
                                     "/"+simtrack_head_node_name+"/"+object_names[i],
                                     100,
                                     boost::bind(simtrack_head_pose_cb, _1, object_names[i]));

        pose_pubs[object_names[i]] = main_node_handle.advertise<geometry_msgs::Pose>(node_name+"/"+object_names[i], 100);

    }


    ros::spin();
    ros::shutdown();
    return 0;
}

#else

int main( int argc, char** argv )
{
    return 0;
}

#endif

