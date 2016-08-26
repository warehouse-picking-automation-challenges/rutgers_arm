#include "ObjectDetect/apc_object.hpp"
#include "prx/utilities/definitions/defs.hpp"

#include <iostream>
#include <vector>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/io/bag_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <geometry_msgs/PoseStamped.h>
#include "prx_utilities/describe_geometries_srv.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>


#define PI 3.14
#define SHELF_NUM_SAMPLES 100

extern tf::TransformListener * world_tf_listener;
extern ros::NodeHandle* PtrNodeHandle;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Pt;

void applyTF(geometry_msgs::PoseStamped SrcPose, geometry_msgs::PoseStamped &DstPose, tf::Transform world_tf);
void applyTF(Eigen::Vector3d SrcPoint, Eigen::Vector3d& DstPoint, tf::Transform transform);
void PrintPose(geometry_msgs::PoseStamped pPose, std::string);
bool get_transform(std::string Targetframe, std::string Srcframe, tf::StampedTransform& transform);
void view2clouds(PointCloud::Ptr &, PointCloud::Ptr &, Eigen::Vector3d, Eigen::Vector3d,
              Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d);
void visualize_point_cloud(PointCloud::Ptr);
void getTransformationMatrix(Eigen::Matrix4f &transform, tf::Matrix3x3 rotation, Eigen::Vector3d origin);
void applyTF(PointCloud::Ptr cloud_in, tf::Matrix3x3 rotation, Eigen::Vector3d origin);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
void testObjectDetect();