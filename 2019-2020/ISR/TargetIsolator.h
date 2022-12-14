#pragma once
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

namespace grip {

/**
* TargetIsolator class.
*
* An OpenCV pipeline generated by GRIP.
*/
class TargetIsolator {
	private:
		cv::Mat blurOutput;
		cv::Mat hsvThreshold0Output;
		cv::Mat cvDilate0Output;
		cv::Mat maskOutput;
		cv::Mat hsvThreshold1Output;
		cv::Mat cvDilate1Output;
		std::vector<std::vector<cv::Point> > findContoursOutput;
		std::vector<std::vector<cv::Point> > filterContoursOutput;
		std::vector<std::vector<cv::Point> > convexHullsOutput;
		void blur(cv::Mat &, double , cv::Mat &);
		void mask(cv::Mat &, cv::Mat &, cv::Mat &);
		void hsvThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void cvDilate(cv::Mat &, cv::Mat &, cv::Point &, double , int , cv::Scalar &, cv::Mat &);
		void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
		void filterContours(std::vector<std::vector<cv::Point> > &, double , double , double , double , double , double , double [], double , double , double , double , std::vector<std::vector<cv::Point> > &);
		void convexHulls(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &);

	public:
		TargetIsolator();
		void Process(cv::Mat& source0, double altitude);
		cv::Mat* GetBlurOutput();
		cv::Mat* GetHsvThreshold0Output();
		cv::Mat* GetCvDilate0Output();
		cv::Mat* GetMaskOutput();
		cv::Mat* GetHsvThreshold1Output();
		cv::Mat* GetCvDilate1Output();
		std::vector<std::vector<cv::Point> >* GetFindContoursOutput();
		std::vector<std::vector<cv::Point> >* GetFilterContoursOutput();
		std::vector<std::vector<cv::Point> > GetConvexHullsOutput();
};


} // end namespace grip
