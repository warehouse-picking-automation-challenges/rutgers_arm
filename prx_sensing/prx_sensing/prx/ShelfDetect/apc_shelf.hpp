#ifndef APC_SHELF
#define APC_SHELF

#include <iostream>
#include <vector>
#include <string>
#include <utility>

#include <prx_sensing.h>

#define BB_SIZE 10
#define OUTLIER_THRESHOLD 10

/*Class to maintain information about shelf used in apc*/
class apc_shelf
{
	public:
		apc_shelf();
		bool GetLeftRightPoses(PointCloud::Ptr);
		int CheckNeighboringBox(PointCloud::Ptr, int);
		bool PlaneEstimate(PointCloud::Ptr);
		bool EdgeEstimate(PointCloud::Ptr, Eigen::Vector3d&, Eigen::Vector3d&);
		geometry_msgs::PoseStamped getDefaultShelfPose();
		bool smoothenShelfEstimate(geometry_msgs::PoseStamped&);
		bool ValidateEstimateWithDefault();
		double getShelfBinHeight(int bin, Eigen::Vector3d center, tf::Matrix3x3 rotation);
		void getShelfBinPoints(PointCloud::Ptr cloud_in, int bin);

		tf::StampedTransform CameraToWorldTf;
		geometry_msgs::PoseStamped defShelfPose;
		int init;

	protected:
		double FOURTH_SHELF_HEIGHT;
		double THIRD_SHELF_HEIGHT;
		double SECOND_SHELF_HEIGHT;
		double FIRST_SHELF_HEIGHT;
		double LEFT_SHELF_WIDTH;
		double MIDDLE_SHELF_WIDTH;
		double RIGHT_SHELF_WIDTH;
		double SHELF_DEPTH;
		double TOP_SHELF_OFFSET;
		double HOR_SHELF_OFFSET;
		double SHELF_MID_HEIGHT;
		double SHELF_MID_WIDTH;
		double SHELF_MID_DEPTH;
		double DEPTH_RANGE_MIN;
		double DEPTH_RANGE_MAX;

		double planeDistance;
		Eigen::Vector3d planeNormal;
    	Eigen::Vector3d EdgeVector;
    	Eigen::Vector3d CrossVector;

    	Eigen::Vector3d LeftTop;
    	Eigen::Vector3d RightTop;
    	Eigen::Vector3d center;
    	tf::Matrix3x3 rotation;

    	std::deque<Eigen::Vector3d > SampledCenters;
    	std::deque<Eigen::Vector4d > SampledRotations;
};

extern apc_shelf* shelf;

#endif