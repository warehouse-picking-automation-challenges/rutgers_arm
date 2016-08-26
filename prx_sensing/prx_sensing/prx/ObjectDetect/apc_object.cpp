#include "apc_object.hpp"
#include <dirent.h>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

apc_object::apc_object()
{
	label = PRX_NONE;
	TrainingMask.clear();
	TrainingImgNames.clear();
	bin.clear();
	ColorHist = Mat::zeros(183, 1, CV_32F);
}

apc_object::apc_object(ObjectLabels Initlabel)
{
	label = Initlabel;
	TrainingMask.clear();
	TrainingImgNames.clear();
	bin.clear();
	ColorHist = Mat::zeros(183, 1, CV_32F);
}

apc_object::~apc_object() { }

void apc_object::LoadData(std::string path)
{
	DIR* pDIR;
	struct dirent *entry;

	std::string location = path + "/" + patch::to_string(label);
	std::cout<<"HERE : "<<location<<std::endl;
	if( pDIR=opendir(location.c_str()) ){
		while(entry = readdir(pDIR)){
			if(entry->d_type == DT_REG){
				std::cout<<entry->d_name<<std::endl;
				cv::Mat image = cv::imread(location + "/" + entry->d_name, CV_8UC1);
				TrainingMask.push_back(image.clone());
				TrainingImgNames.push_back(entry->d_name);
			}
		}
		closedir(pDIR);
	}
}

void apc_object::ShowTrainingImages(std::string path)
{
	const std::string winName = "image";

	for(int i=0;i<TrainingImgNames.size();i++){
		cv::Mat res;
		cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
		std::string location = "home/pracsys/pracsys/src/prx_sensing/prx_sensing/prx/training/data/images/" + TrainingImgNames[i];
		cv::Mat image = cv::imread(location, CV_LOAD_IMAGE_COLOR);
		image.copyTo(res, TrainingMask[i]);
		cv::imshow(winName, res);
		cv::waitKey(0);
		cv::destroyWindow(winName);
	}
}

void apc_object::getColorFeatures()
{
	for(int idx=0;idx<TrainingImgNames.size();idx++){
		cv::Mat hsv_image;

		std::string location = "/home/pracsys/pracsys/src/prx_sensing/prx_sensing/prx/training/data/images/" + TrainingImgNames[idx];
		cv::Mat image = cv::imread(location, CV_LOAD_IMAGE_COLOR);
		cv::Mat hwgb_image = ConvertImgtoHWGB(image);
		cv::Mat hist = calcHistogram(hwgb_image, TrainingMask[idx], 0, 182);
		ColorHist = ColorHist + hist;
	}
	std::cout<<ColorHist<<std::endl;
}

void apc_object::getShelfHeightFeature()
{
  Eigen::Vector3d center(1.165, 0.0, 1.11);
  tf::Matrix3x3 rotation;
  rotation.setIdentity();
	const std::string winName = "image";
	for(int idx=0;idx<TrainingImgNames.size();idx++){
		std::string location = "/home/pracsys/pracsys/src/prx_sensing/prx_sensing/prx/training/data/images/" + TrainingImgNames[idx];
		std::string str = "depth";
		location.replace(0,5,str);
		cv::Mat image = cv::imread(location, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
		image.convertTo(image, CV_32F);
		cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
		cv::imshow(winName, image);
		cv::waitKey(0);
		cv::destroyWindow(winName);
		double height = shelf->getShelfBinHeight(bin[idx], center, rotation);
		cv::Mat heightImg = ConvertImgtoHeightImg(image);
	}
}

Mat apc_object::calcHistogram(Mat image, Mat mask, int range_low, int range_high)
{
	Mat hist;
	int hbins = range_high - range_low + 1;
    int histSize[] = {hbins};
    float hranges[] = { range_low, range_high };
    const float* ranges[] = { hranges };
    int channels[] = {0};
    calcHist( &image, 1, channels, mask,
             hist, 1, histSize, ranges,
             true,
             false );
    return hist;
}

Mat apc_object::ConvertImgtoHeightImg(Mat image)
{
	double fx_d = 1.0 / 1051.89;
	double fy_d = 1.0 / 1060.192;
	double cx_d = 962.20;
	double cy_d = 535.165;

	cv::Mat height_image = Mat::zeros(image.rows, image.cols, CV_64F);

	for(int i=0; i<image.rows; i++)
		for(int j=0; j<image.cols; j++){
			double depth = image.at<double>(i,j);
			Eigen::Vector3d point(  ((j - cx_d) * depth * fx_d), 
									((i - cy_d) * depth * fy_d),
									depth );
			applyTF(point, point, shelf->CameraToWorldTf);
			image.at<double>(i,j) = point[2];
		}

    return height_image;
}

Mat apc_object::ConvertImgtoHWGB(Mat image)
{
	cv::Mat hsv_image;
	
	cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
	std::vector<cv::Mat> hsv_channels;
	cv::split(hsv_image, hsv_channels);
	cv::Mat hwgb_image = Mat::zeros(hsv_image.rows, hsv_image.cols, CV_8UC1);

	for(int i=0; i<hsv_image.rows; i++)
		for(int j=0; j<hsv_image.cols; j++){
			int hue = (int)hsv_channels[0].at<uchar>(i,j);
			int sat = (int)hsv_channels[1].at<uchar>(i,j);
			int val = (int)hsv_channels[2].at<uchar>(i,j);

			if( sat > 90 && val > 20 )
				hwgb_image.at<uchar>(i,j) = hue;
			else if(val > 200)
				hwgb_image.at<uchar>(i,j) = HUE_WHITE;
			else if(val > 50 && val < 200)
				hwgb_image.at<uchar>(i,j) = HUE_GRAY;
			else if(val < 50)
				hwgb_image.at<uchar>(i,j) = HUE_BLACK;
		}

    return hwgb_image;
}

void apc_object::getNormalCurvature()
{
	for(int idx=0;idx<TrainingImgNames.size();idx++){
		std::string str = TrainingImgNames[idx];
		str.replace(0,5,"depth");
		std::string location = "/home/pracsys/pracsys/src/prx_sensing/prx_sensing/prx/training/data/images/" + str;
		std::cout<<location<<std::endl;
		cv::Mat image = cv::imread(location, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
		image.convertTo(image, CV_32F);
		cv::Mat NormalImg = ConvertImgtoNormalImg(image, TrainingMask[idx]);
		//cv::Mat hist = calcHistogram(NormalImg, TrainingMask[idx], 0, 182);
		//NormalHist = NormalHist + hist;
	}
}

Mat apc_object::ConvertImgtoNormalImg(Mat image, Mat mask)
{
	double fx_d = 1.0 / 1051.89;
	double fy_d = 1.0 / 1060.192;
	double cx_d = 962.20;
	double cy_d = 535.165;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	cv::Mat normal_image = Mat::zeros(image.rows, image.cols, CV_8UC1);

	for(int i=0; i<image.rows; i++)
		for(int j=0; j<image.cols; j++){
			pcl::PointXYZ point;
			double depth = image.at<float>(i,j);
			if(depth != 0)
			{
				point.x = ((j - cx_d) * depth * fx_d);
				point.y = ((i - cy_d) * depth * fy_d);
				point.z = depth;
				cloud->points.push_back(point);
			}
			 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    		 viewer = simpleVis(cloud);
    		 while (!viewer->wasStopped ())
			  {
			    viewer->spinOnce (100);
			    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			  }
			//std::cout<<"Point val : "<<i<<" "<<j<<" "<<val<<std::endl;
		}

    return normal_image;	
}



