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

#include "apc_shelf.hpp"

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include "prx/utilities/definitions/defs.hpp"

#include <pcl/segmentation/sac_segmentation.h>

//Initialize shelf parameters
apc_shelf::apc_shelf()
{
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/FOURTH_SHELF_HEIGHT", FOURTH_SHELF_HEIGHT);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/THIRD_SHELF_HEIGHT", THIRD_SHELF_HEIGHT);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/SECOND_SHELF_HEIGHT", SECOND_SHELF_HEIGHT);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/FIRST_SHELF_HEIGHT", FIRST_SHELF_HEIGHT);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/LEFT_SHELF_WIDTH", LEFT_SHELF_WIDTH);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/MIDDLE_SHELF_WIDTH", MIDDLE_SHELF_WIDTH);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/RIGHT_SHELF_WIDTH", RIGHT_SHELF_WIDTH);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/SHELF_DEPTH", SHELF_DEPTH);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/TOP_SHELF_OFFSET", TOP_SHELF_OFFSET);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/HOR_SHELF_OFFSET", HOR_SHELF_OFFSET);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEPTH_RANGE_MIN", DEPTH_RANGE_MIN);
    ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEPTH_RANGE_MAX", DEPTH_RANGE_MAX);
    
    SHELF_MID_HEIGHT = TOP_SHELF_OFFSET + (FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT + SECOND_SHELF_HEIGHT + FIRST_SHELF_HEIGHT)/2;
    SHELF_MID_WIDTH = HOR_SHELF_OFFSET + (LEFT_SHELF_WIDTH + MIDDLE_SHELF_WIDTH + RIGHT_SHELF_WIDTH)/2;
    SHELF_MID_DEPTH = SHELF_DEPTH/2;
    init = 0;
}

//Get Default Shelf Pose
geometry_msgs::PoseStamped apc_shelf::getDefaultShelfPose()
{
    if(init == 0)
    {
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_QX", defShelfPose.pose.orientation.x);
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_QY", defShelfPose.pose.orientation.y);
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_QZ", defShelfPose.pose.orientation.z);
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_QW", defShelfPose.pose.orientation.w);
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_X", defShelfPose.pose.position.x);
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_Y", defShelfPose.pose.position.y);
      ros::param::get("/planning/world_model/simulator/obstacles/shelf/DEFULT_Z", defShelfPose.pose.position.z);
    }

    return defShelfPose;
}

//Edge Estimation RANSAC
bool apc_shelf::EdgeEstimate(PointCloud::Ptr cloud, Eigen::Vector3d &inlierPoint, Eigen::Vector3d &shelfVector)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.001);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size () == 0){
        PRX_WARN_S("prx_sensing : No Inliers in Edge Estimation");
        return false;
    }

    //Get the Edge vector in the plane of the shelf
    shelfVector[0] = coefficients->values[3] - (coefficients->values[3] * planeNormal[0]);
    shelfVector[1] = coefficients->values[4] - (coefficients->values[4] * planeNormal[1]);
    shelfVector[2] = coefficients->values[5] - (coefficients->values[5] * planeNormal[2]);

    double norm = shelfVector.norm();
    shelfVector = shelfVector/norm;

    //Get one point on the edge
    inlierPoint << coefficients->values[0], 
            coefficients->values[1], coefficients->values[2];

    double distance = inlierPoint[0]*planeNormal[0] +
                        inlierPoint[1]*planeNormal[1] +
                        inlierPoint[2]*planeNormal[2] + planeDistance;

    //Return the projection of an inlier on the plane
    inlierPoint = inlierPoint - (distance * planeNormal);

    return true;
}

//RANSAC for Plane Fitting
bool apc_shelf::PlaneEstimate(PointCloud::Ptr cloud)
{
    std::cout<<cloud->points.size()<<std::endl;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0){
        PRX_WARN_S("prx_sensing : No Inliers in Plane Estimation");
        return false;
    }

    planeNormal << coefficients->values[0], coefficients->values[1],
                    coefficients->values[2];
    planeDistance = coefficients->values[3];

    return true;
}

//Check if the point is an outlier
int apc_shelf::CheckNeighboringBox(PointCloud::Ptr temp_cloud, int pos)
{
  int wid = temp_cloud->width;
  int ht = temp_cloud->height;
  int topleft = (pos -BB_SIZE) - (BB_SIZE*wid);
  int botleft = topleft + (2*BB_SIZE*wid);
  std::vector<int> res;

  for(int i=topleft;i<botleft;i=i+wid){
    int start = i;
    int end = i + 2*BB_SIZE;
    for(int j=start;j<end;j++){
        if(j<=0 || j>= wid*ht)continue;
        double z = (double)temp_cloud->points[j].z;
        if(z > DEPTH_RANGE_MIN && z< DEPTH_RANGE_MAX){
           res.push_back(j);
           break;
      }
    }
  }
  
  if(res.size() > OUTLIER_THRESHOLD)return 1;
  else return 0;
}

//Finding leftmost and rightmost point within depth limit for each row iterating through the whole image
bool apc_shelf::GetLeftRightPoses(PointCloud::Ptr temp_cloud)
{
    PointCloud::Ptr cloud(new PointCloud);
    
    int wid = temp_cloud->width;
    int ht = temp_cloud->height;

    //Extract point clouds for left edge
    PointCloud::Ptr lcloud(new PointCloud);
    for(int i=0;i<ht;i++){
      for(int j=0;j<wid;j++){
        int pos = i*wid + j;
        double z = (double)temp_cloud->points[pos].z;
        if(z > DEPTH_RANGE_MIN && z< DEPTH_RANGE_MAX){
          if(CheckNeighboringBox(temp_cloud, pos) == 1){
            cloud->points.push_back(temp_cloud->points[pos]);
            lcloud->points.push_back(temp_cloud->points[pos]);
            break;
          }//Check Neighbors
        }//Check depth limits
      }//Iterate over width
    }//Iterate over height

    //Extract point clouds for right edge
    PointCloud::Ptr rcloud(new PointCloud);
    for(int i=0;i<ht;i++){
      for(int j=wid-1;j>0;j--){
        int pos = i*wid + j;
        double z = (double)temp_cloud->points[pos].z;
        if(z > DEPTH_RANGE_MIN && z< DEPTH_RANGE_MAX){
          if(CheckNeighboringBox(temp_cloud, pos) == 1){
            cloud->points.push_back(temp_cloud->points[pos]);
            rcloud->points.push_back(temp_cloud->points[pos]);
            break;
          }//Check Neighbors
        }//Check depth limits
      }//Iterate over width
    }//Iterate over height

    //Get Point cloud for top edge
    PointCloud::Ptr tcloud(new PointCloud);
    for(int i=0;i<wid;i++){
      for(int j=0;j<ht;j++){
        int pos = j*wid + i;
        double z = (double)temp_cloud->points[pos].z;
        if(z > DEPTH_RANGE_MIN && z< DEPTH_RANGE_MAX){
          if(CheckNeighboringBox(temp_cloud, pos) == 1){
            cloud->points.push_back(temp_cloud->points[pos]);
            tcloud->points.push_back(temp_cloud->points[pos]);
            break;
          }//Check Neighbors
        }//Check depth limits
      }//Iterate over the height
    }//Iterate over the width

    //Plane Estimation
    if(PlaneEstimate(cloud) == 0)
      return false;

    //Edge Estimations
    Eigen::Vector3d LeftEdge;
    Eigen::Vector3d leftInlier;
    if(EdgeEstimate(lcloud, leftInlier, LeftEdge) == 0)
      return false;

    Eigen::Vector3d RightEdge;
    Eigen::Vector3d rightInlier;
    if(EdgeEstimate(rcloud, rightInlier, RightEdge) == 0)
      return false;

    Eigen::Vector3d TopEdge;
    Eigen::Vector3d topInlier;
    if(EdgeEstimate(tcloud, topInlier, TopEdge) == 0)
      return false;

    //Averaging the two edge vectors
    EdgeVector = (LeftEdge + RightEdge)/2;

    //Cross Product to get the third vector
    CrossVector = EdgeVector.cross(planeNormal);

    //Find the Intersection
    double distl = (leftInlier[1]*LeftEdge[0] + topInlier[0] - leftInlier[0] - LeftEdge[0]*topInlier[1])/(CrossVector[1]*LeftEdge[0] - CrossVector[0]);
    double distr = (rightInlier[1]*RightEdge[0] + topInlier[0] - rightInlier[0] - RightEdge[0]*topInlier[1])/(CrossVector[1]*RightEdge[0] - CrossVector[0]);
    
    LeftTop = topInlier + distl*CrossVector;
    RightTop = topInlier + distr*CrossVector;

    //To view the estimates
    //view2clouds(cloud, temp_cloud, LeftTop, RightTop, planeNormal, EdgeVector, CrossVector);

    applyTF(LeftTop, LeftTop, CameraToWorldTf);
    applyTF(RightTop, RightTop, CameraToWorldTf);

    //get orientation
    tf::Quaternion shelf_rotation, camera_tf;
    double roll, pitch, yaw;
    rotation.setValue(planeNormal[0], CrossVector[0], EdgeVector[0],
            planeNormal[1], CrossVector[1], EdgeVector[1],
            planeNormal[2], CrossVector[2], EdgeVector[2]);
    rotation.getRotation(shelf_rotation);
    camera_tf = CameraToWorldTf.getRotation();
    shelf_rotation = camera_tf*shelf_rotation;
    tf::Matrix3x3(shelf_rotation).getRPY(roll, pitch, yaw);
    rotation.setRPY(0.0, 0.0, yaw);

    Eigen::Vector3d centerl,centerr;
    centerl[0] = LeftTop[0] + (rotation.getColumn(0).getX())*SHELF_MID_DEPTH;
    centerl[1] = LeftTop[1] - (rotation.getColumn(1).getY())*SHELF_MID_WIDTH;
    centerl[2] = LeftTop[2] - (rotation.getColumn(2).getZ())*SHELF_MID_HEIGHT;

    centerr[0] = RightTop[0] + (rotation.getColumn(0).getX())*SHELF_MID_DEPTH;
    centerr[1] = RightTop[1] + (rotation.getColumn(1).getY())*SHELF_MID_WIDTH;
    centerr[2] = RightTop[2] - (rotation.getColumn(2).getZ())*SHELF_MID_HEIGHT;
    
    center = (centerl + centerr)/2;
    
    std::cout<<"Raw Estimate :: Center "<<center[0]<<" "<<center[1]<<" "<<center[2]<<std::endl;
    std::cout<<"Raw Estimate :: RPY "<<roll*180/PI<<" "<<pitch<<" "<<yaw<<std::endl;
    
    return true;
}

bool apc_shelf::ValidateEstimateWithDefault()
{
    geometry_msgs::PoseStamped defaultShelfPose = this->getDefaultShelfPose();
    Eigen::Vector3d defaultCenter(defaultShelfPose.pose.position.x,
                                    defaultShelfPose.pose.position.y,
                                    defaultShelfPose.pose.position.z);
    double distance = (center - defaultCenter).squaredNorm();
    double roll,pitch,yaw;
    rotation.getRPY(roll, pitch, yaw);
    roll = roll*180/PI;pitch = pitch*180/PI;yaw = yaw*180/PI;

    if(distance > 0.2 || abs(roll) > 10 || abs(yaw) > 10)return false;
    else return true;
}

bool apc_shelf::smoothenShelfEstimate(geometry_msgs::PoseStamped &ResultPose)
{
    if(this->ValidateEstimateWithDefault() == 0)return false;

    tf::Quaternion sampleq;
    rotation.getRotation(sampleq);
    Eigen::Vector4d rot4d(sampleq[0], sampleq[1], sampleq[2], sampleq.getW());

    if(SampledCenters.size() < SHELF_NUM_SAMPLES){
            SampledCenters.push_back(center);
            SampledRotations.push_back(rot4d);
    }
     else{
         SampledCenters.pop_front();
         SampledRotations.pop_front();
         SampledCenters.push_back(center);
         SampledRotations.push_back(rot4d);
     }

     Eigen::Vector3d meanInliersCenter;
     Eigen::Vector4d meanInliersRotation;
     int maximumCountCenter = 0;
     int maximumCountRotation = 0;
     for(int i=0;i<SampledCenters.size();i++)
     {
        Eigen::Vector3d point = SampledCenters[i];
        Eigen::Vector4d rotpoint = SampledRotations[i];

        meanInliersCenter = point;
        meanInliersRotation = rotpoint;
        int countCenter = 1;
        int countRotation = 1;
        for(int j=0;j<SampledCenters.size(), j!=i; j++)
        {
            if((point - SampledCenters[j]).squaredNorm() < 0.02){
                countCenter++;
                meanInliersCenter += point;
            }
            if((rotpoint - SampledRotations[j]).squaredNorm() < 0.2){
                countRotation++;
                meanInliersRotation += rotpoint;
            }
        }

        if(countCenter > maximumCountCenter){
            maximumCountCenter = countCenter;
            meanInliersCenter /= countCenter;
        }

        if(countRotation > maximumCountRotation){
            maximumCountRotation = countRotation;
            meanInliersRotation /= countRotation;
        }
     }
     center = meanInliersCenter;
     tf::Quaternion meanq(meanInliersRotation[0],
                             meanInliersRotation[1],
                             meanInliersRotation[2],
                             meanInliersRotation[3]);
     meanq.normalize();
     rotation.setRotation(meanq);

     ResultPose.pose.position.x = center[0];
     ResultPose.pose.position.y = center[1];
     ResultPose.pose.position.z = center[2];
     ResultPose.pose.orientation.x = meanq[0];
     ResultPose.pose.orientation.y = meanq[1];
     ResultPose.pose.orientation.z = meanq[2];
     ResultPose.pose.orientation.w = meanq.getW();

     PrintPose(ResultPose,"After Filter");
     std::cout<<"#########################################"<<std::endl;
     return true;
}

double apc_shelf::getShelfBinHeight(int bin, Eigen::Vector3d center, tf::Matrix3x3 rotation)
{
  Eigen::Vector3d LEFT_TOP;
  LEFT_TOP[0] = center[0] - (rotation.getColumn(0).getX())*SHELF_MID_DEPTH;
  LEFT_TOP[1] = center[1] + (rotation.getColumn(1).getY())*SHELF_MID_WIDTH;
  LEFT_TOP[2] = center[2] + (rotation.getColumn(2).getZ())*SHELF_MID_HEIGHT;

  if(bin == 1 || bin ==2 || bin ==3)
    return (LEFT_TOP[2] - 
            (rotation.getColumn(2).getZ())*(FOURTH_SHELF_HEIGHT + TOP_SHELF_OFFSET));
  else if(bin == 4 || bin == 5 || bin == 6)
    return (LEFT_TOP[2] - 
            (rotation.getColumn(2).getZ())*(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT + TOP_SHELF_OFFSET));
  else if(bin == 7 || bin == 8 || bin == 9)
    return (LEFT_TOP[2] - 
            (rotation.getColumn(2).getZ())*(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT + SECOND_SHELF_HEIGHT + TOP_SHELF_OFFSET));
  else if(bin == 10 || bin == 11 || bin == 12)
    return (LEFT_TOP[2] - 
            (rotation.getColumn(2).getZ())*(SHELF_MID_HEIGHT*2));
}

void apc_shelf::getShelfBinPoints(PointCloud::Ptr cloud_in, int bin)
{
  PointCloud::Ptr cloud_out(new PointCloud);

  Eigen::Vector3d origin;
  Eigen::Vector3d LEFT_TOP;
  //Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Vector3d OFFSET(0.0,0.0,0.0);
  double MIN_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z;
  double LEG_WIDTH = 0.05;
  double LIP_HEIGHT = 0.05;

  LEFT_TOP[0] = center[0] - (rotation.getColumn(0).getX())*SHELF_MID_DEPTH;
  LEFT_TOP[1] = center[1] + (rotation.getColumn(1).getY())*SHELF_MID_WIDTH - LEG_WIDTH;
  LEFT_TOP[2] = center[2] + (rotation.getColumn(2).getZ())*SHELF_MID_HEIGHT - LIP_HEIGHT;

  if(bin == 1)
  {
    OFFSET << 0.0,0.0,0.0;

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(LEFT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(FOURTH_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;
  }
  else if(bin == 2)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH - LEG_WIDTH/2),0.0;

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -MIDDLE_SHELF_WIDTH; MAX_Y = 0;
    MIN_Z = -(FOURTH_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;
  }
  else if(bin == 3)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH + MIDDLE_SHELF_WIDTH - LEG_WIDTH/2),0.0;

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(RIGHT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(FOURTH_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }
  else if(bin == 4)
  {
    OFFSET << 0.0, 0.0,-FOURTH_SHELF_HEIGHT;

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(LEFT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(THIRD_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }
  else if(bin == 5)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH - LEG_WIDTH/2),-FOURTH_SHELF_HEIGHT;

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -MIDDLE_SHELF_WIDTH; MAX_Y = 0;
    MIN_Z = -(THIRD_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0; 
  }
  else if(bin == 6)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH + MIDDLE_SHELF_WIDTH - LEG_WIDTH/2),-FOURTH_SHELF_HEIGHT;

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(RIGHT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(THIRD_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;
  }

  else if(bin == 7)
  {
    OFFSET << 0.0, 0.0,-(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT);

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(LEFT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(SECOND_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }
  else if(bin == 8)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH - LEG_WIDTH/2),-(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT);

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -MIDDLE_SHELF_WIDTH; MAX_Y = 0;
    MIN_Z = -(SECOND_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }
  else if(bin == 9)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH + MIDDLE_SHELF_WIDTH - LEG_WIDTH/2),-(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT);

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(RIGHT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(SECOND_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }

  else if(bin == 10)
  {
    OFFSET << 0.0, 0.0,-(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT + SECOND_SHELF_HEIGHT);

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(LEFT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(FIRST_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }
  else if(bin == 11)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH - LEG_WIDTH/2),-(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT + SECOND_SHELF_HEIGHT);

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -MIDDLE_SHELF_WIDTH; MAX_Y = 0;
    MIN_Z = -(FIRST_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }
  else if(bin == 12)
  {
    OFFSET << 0.0, -(LEFT_SHELF_WIDTH + MIDDLE_SHELF_WIDTH - LEG_WIDTH/2),-(FOURTH_SHELF_HEIGHT + THIRD_SHELF_HEIGHT + SECOND_SHELF_HEIGHT);

    MIN_X = 0; MAX_X = SHELF_MID_DEPTH;
    MIN_Y = -(RIGHT_SHELF_WIDTH - LEG_WIDTH/2); MAX_Y = 0;
    MIN_Z = -(FIRST_SHELF_HEIGHT - LIP_HEIGHT/2); MAX_Z = 0;  
  }


  origin[0] = LEFT_TOP[0] + (rotation.getColumn(0).getX())*OFFSET[0];
  origin[1] = LEFT_TOP[1] + (rotation.getColumn(1).getY())*OFFSET[1];
  origin[2] = LEFT_TOP[2] + (rotation.getColumn(2).getZ())*OFFSET[2];

  applyTF(cloud_in, rotation.inverse(), -origin);

  // getTransformationMatrix(transform, rotation.inverse(), -origin);
  // pcl::transformPointCloud(*cloud_in, *cloud_out, transform);
  // pcl::copyPointCloud(*cloud_out, *cloud_in);

  //std::cout << transform.matrix() << std::endl;
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (MIN_X, MAX_X);
  pass.filter (*cloud_in);

  pcl::PassThrough<pcl::PointXYZRGB> pass2;
  pass2.setInputCloud (cloud_in);
  pass2.setFilterFieldName ("y");
  pass2.setFilterLimits (MIN_Y, MAX_Y);
  pass2.filter (*cloud_in);

  pcl::PassThrough<pcl::PointXYZRGB> pass3;
  pass3.setInputCloud (cloud_in);
  pass3.setFilterFieldName ("z");
  pass3.setFilterLimits (MIN_Z, MAX_Z);
  pass3.filter (*cloud_in);

  applyTF(cloud_in, rotation, origin);
}