#ifndef APC_OBJECT
#define APC_OBJECT

#include <prx_sensing.h>
#include <apc_shelf.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace cv;

/*Enumerated Lables for each obect*/
enum ObjectLabels {
	PRX_NONE,
	PRX_DUMBBELLS,
	PRX_SQUEKING_EGGS,
	PRX_BOTTLE
};

#define HUE_WHITE 180
#define HUE_GRAY 181
#define HUE_BLACK 182

/*Class to maintain information about objects used in apc*/
class apc_object
{
	public:
		apc_object();
		apc_object(ObjectLabels label);
		~apc_object();

		void LoadData(std::string path);
		void ShowTrainingImages(std::string path);
		void ConvertHSVtoHWGB();
		cv::Mat ConvertImgtoHWGB(Mat image);
		cv::Mat calcHistogram(Mat image, Mat mask, int range_low, int rangle_high);
		void getColorFeatures();
    	void getShelfHeightFeature();
    	Mat ConvertImgtoHeightImg(Mat image);
    	void getNormalCurvature();
    	Mat ConvertImgtoNormalImg(Mat image, Mat mask);


	protected:
		ObjectLabels label;
		std::vector<cv::Mat> TrainingMask;
		std::vector<std::string> TrainingImgNames;
		std::vector<int> bin;

		cv::Mat ColorHist;
		cv::Mat ShelfHeightHist;
};

#endif
