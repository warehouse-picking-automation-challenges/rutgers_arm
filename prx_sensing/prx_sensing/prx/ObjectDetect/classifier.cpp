#include <apc_shelf.hpp>
#include <apc_object.hpp>
#include <pcl/point_types_conversion.h>

void extractFeatures(PointCloud cloud_xyzrgb, int bin)
{	
	//Estimating the normals
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
	pcl::copyPointCloud(cloud_xyzrgb, *cloud_xyz);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud_xyz);
    normal_estimator.setRadiusSearch(0.005);
    normal_estimator.compute (*normals);

	for(int pos = 0;pos<cloud_xyzrgb.points.size();pos++){
		Eigen::Vector2d feature;
		
		//Get the color feature
		pcl::PointXYZHSV HSVPoint;
		pcl::PointXYZRGBtoXYZHSV(cloud_xyzrgb.points[pos], HSVPoint);
		int hue = HSVPoint.h;
		int sat = HSVPoint.s;
		int val = HSVPoint.v;

		if( sat > 90 && val > 20 )
			feature << hue;
		else if(val > 200)
			feature << HUE_WHITE;
		else if(val > 50 && val < 200)
			feature << HUE_GRAY;
		else if(val < 50)
			feature << HUE_BLACK;

		//Get height from shelf
		double shelf_height = shelf->getShelfBinHeight(bin, shelf->center, shelf->rotation);
		Eigen::Vector3d point(cloud_xyzrgb.points[pos].x, cloud_xyzrgb.points[pos].y,
									cloud_xyzrgb.points[pos].z);
		applyTF(point, point, shelf->CameraToWorldTf);
		feature << point[2] - shelf_height;

		//Get the Curvature from Normal
 		feature << (*normals)[pos].curvature;
	}
}

PointCloud extractPointCloud(PointCloud cloud_xyzrgb, int bin)
{
	
}