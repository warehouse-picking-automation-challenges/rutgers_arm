#include "prx_sensing.h"

void visualize_point_cloud(PointCloud::Ptr current_cloud)
{
    ros::ServiceClient pointCloudVis_cl = (*PtrNodeHandle).serviceClient<prx_utilities::describe_geometries_srv>("/visualization/geometries");
    pointCloudVis_cl.waitForExistence(ros::Duration(-1.0));
    prx_utilities::describe_geometries_srv srv_cloud;

    prx_utilities::rigid_body_info_msg msg;
    msg.system_name = "points";
    msg.rigid_body_name = "pointCloud";
    msg.material_name = "white";
    msg.geometry_type = 12;

    for(int i=0;i<current_cloud->points.size();i++)
    {
        msg.geometry_params.push_back(current_cloud->points[i].x);
        msg.geometry_params.push_back(current_cloud->points[i].y);
        msg.geometry_params.push_back(current_cloud->points[i].z);
        
        double blue, red, green;
        uint32_t rgb_val_;
        memcpy(&rgb_val_, &(current_cloud->points[i].rgb), sizeof(uint32_t));
        blue=rgb_val_ & 0x000000ff;
        rgb_val_ = rgb_val_ >> 8;
        green=rgb_val_ & 0x000000ff;
        rgb_val_ = rgb_val_ >> 8;
        red=rgb_val_ & 0x000000ff;
        msg.geometry_params.push_back(red);
        msg.geometry_params.push_back(green);
        msg.geometry_params.push_back(blue);
    }

    srv_cloud.request.rigid_body_array.push_back(msg);

    pointCloudVis_cl.call(srv_cloud);
}

bool get_transform(std::string Targetframe, std::string Srcframe, tf::StampedTransform& transform)
{
    bool tf_available = false;
    tf_available = world_tf_listener->waitForTransform(Targetframe, Srcframe, 
                                                  ros::Time(0),ros::Duration(0.1));
    if (tf_available)
    {
        try {
                 world_tf_listener->lookupTransform(Targetframe, Srcframe, 
                                                     ros::Time(0), transform);
        }
        catch (tf::TransformException except) {
                PRX_WARN_S("prx_sensing found no TF from "
                            << Targetframe.c_str() << " to " << Srcframe.c_str()
                            << " -> " << except.what());
                return false;
        }

        return true;
    }
    else
        return false;
}

void getTransformationMatrix(Eigen::Matrix4f &transform, tf::Matrix3x3 rotation, Eigen::Vector3d origin)
{
    transform(0,3) = origin[0];
    transform(1,3) = origin[1];
    transform(2,3) = origin[2];
    transform(0,0) = rotation[0][0];
    transform(0,1) = rotation[0][1];
    transform(0,2) = rotation[0][2];
    transform(1,0) = rotation[1][0];
    transform(1,1) = rotation[1][1];
    transform(1,2) = rotation[1][2];
    transform(2,0) = rotation[2][0];
    transform(2,1) = rotation[2][1];
    transform(2,2) = rotation[2][2];
}

//Apply TF to Point CLoud
void applyTF(PointCloud::Ptr cloud_in, tf::Matrix3x3 rotation, Eigen::Vector3d origin)
{
    tf::Quaternion rot;
    rotation.getRotation(rot);
    tf::Vector3 trans(origin[0],origin[1],origin[2]);
    tf::Transform tf(rot, trans);

    for(int i=0;i<cloud_in->points.size();i++)
    {
        Eigen::Vector3d Point(cloud_in->points[i].x, 
                cloud_in->points[i].y, cloud_in->points[i].z);

        applyTF(Point, Point, tf);
        cloud_in->points[i].x = Point[0];
        cloud_in->points[i].y = Point[1];
        cloud_in->points[i].z = Point[2];
    }
}

//For Pose
void applyTF(geometry_msgs::PoseStamped SrcPose, geometry_msgs::PoseStamped &DstPose, tf::Transform transform)
{
    tf::Quaternion obj_rot(SrcPose.pose.orientation.x,
                                 SrcPose.pose.orientation.y,
                                 SrcPose.pose.orientation.z,
                                 SrcPose.pose.orientation.w);

    tf::Vector3 obj_trans(SrcPose.pose.position.x,
                             SrcPose.pose.position.y,
                             SrcPose.pose.position.z);

    tf::Transform obj_tf(obj_rot, obj_trans);

    obj_tf = transform * obj_tf;
    obj_trans = obj_tf.getOrigin();
    obj_rot = obj_tf.getRotation();

    DstPose.pose.position.x = obj_trans.getX();
    DstPose.pose.position.y = obj_trans.getY();
    DstPose.pose.position.z = obj_trans.getZ();
    DstPose.pose.orientation.x = obj_rot[0];
    DstPose.pose.orientation.y = obj_rot[1];
    DstPose.pose.orientation.z = obj_rot[2];
    DstPose.pose.orientation.w = obj_rot.getW();
}

//For point
void applyTF(Eigen::Vector3d SrcPoint, Eigen::Vector3d& DstPoint, tf::Transform transform)
{
    tf::Quaternion obj_rot(0,0,0,1);

    tf::Vector3 obj_trans(SrcPoint[0], SrcPoint[1], SrcPoint[2]);

    tf::Transform obj_tf(obj_rot, obj_trans);

    obj_tf = transform * obj_tf;
    obj_trans = obj_tf.getOrigin();

    DstPoint << obj_trans.getX(), obj_trans.getY(), obj_trans.getZ();
}

void PrintPose(geometry_msgs::PoseStamped pPose, std::string str)
{
    std::cout<<str<<" : [ "<<pPose.pose.position.x<<" "<<
                pPose.pose.position.y<<" "<<
                pPose.pose.position.z<<" "<<
                pPose.pose.orientation.x<<" "<<
                pPose.pose.orientation.y<<" "<<
                pPose.pose.orientation.z<<" "<<
                pPose.pose.orientation.w<<" ]"<<std::endl;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

// Open 3D viewer and add 2 point clouds with different color,
// Also add edge and plane vectors, and top detected points for shelf calibration
// void view2clouds(PointCloud::Ptr &cloud1, 
//               PointCloud::Ptr &cloud2,
//               Eigen::Vector3d ltop,
//               Eigen::Vector3d rtop,
//               Eigen::Vector3d Normal,
//               Eigen::Vector3d Edge,
//               Eigen::Vector3d Cross)
// {
//   double AXIS_LENGTH = 0.2;
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_two_clouds (new pcl::visualization::PCLVisualizer("3D Viewer"));
//   viewer_two_clouds->setBackgroundColor(0,0,0);

//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1 (cloud1, 0, 255, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2 (cloud2, 255, 0, 0);
//   pcl::PointXYZ lt(ltop[0],ltop[1],ltop[2]);
//   pcl::PointXYZ rt(rtop[0],rtop[1],rtop[2]);

//   pcl::PointXYZ ltedge(ltop[0]+Edge[0]*AXIS_LENGTH,ltop[1]+Edge[1]*AXIS_LENGTH,ltop[2]+Edge[2]*AXIS_LENGTH);
//   pcl::PointXYZ ltshelf(ltop[0]+Cross[0]*AXIS_LENGTH,ltop[1]+Cross[1]*AXIS_LENGTH,ltop[2]+Cross[2]*AXIS_LENGTH);
//   pcl::PointXYZ ltplane(ltop[0]+Normal[0]*AXIS_LENGTH,ltop[1]+Normal[1]*AXIS_LENGTH,ltop[2]+Normal[2]*AXIS_LENGTH);

//   pcl::PointXYZ rtedge(rtop[0]+Edge[0]*AXIS_LENGTH,rtop[1]+Edge[1]*AXIS_LENGTH,rtop[2]+Edge[2]*AXIS_LENGTH);
//   pcl::PointXYZ rtshelf(rtop[0]+Cross[0]*AXIS_LENGTH,rtop[1]+Cross[1]*AXIS_LENGTH,rtop[2]+Cross[2]*AXIS_LENGTH);
//   pcl::PointXYZ rtplane(rtop[0]+Normal[0]*AXIS_LENGTH,rtop[1]+Normal[1]*AXIS_LENGTH,rtop[2]+Normal[2]*AXIS_LENGTH);

//   viewer_two_clouds->addPointCloud<pcl::PointXYZ> (cloud2, color2, "cloud_2");
//   viewer_two_clouds->addPointCloud<pcl::PointXYZ> (cloud1, color1, "cloud_1");

//   viewer_two_clouds->addLine<pcl::PointXYZ> (lt, ltedge, "line1");
//   viewer_two_clouds->addLine<pcl::PointXYZ> (lt, ltshelf, "line2");
//   viewer_two_clouds->addLine<pcl::PointXYZ> (lt, ltplane, "line3");

//   viewer_two_clouds->addLine<pcl::PointXYZ> (rt, rtedge, "line4");
//   viewer_two_clouds->addLine<pcl::PointXYZ> (rt, rtshelf, "line5");
//   viewer_two_clouds->addLine<pcl::PointXYZ> (rt, rtplane, "line6");

//   viewer_two_clouds->initCameraParameters();

//   while(!viewer_two_clouds->wasStopped())
//   {
//       viewer_two_clouds->spinOnce();
//       boost::this_thread::sleep (boost::posix_time::microseconds(100000));
//   }

//   viewer_two_clouds->close();
// }