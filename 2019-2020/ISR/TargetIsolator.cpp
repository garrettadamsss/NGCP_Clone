#include "TargetIsolator.h"

namespace grip {

TargetIsolator::TargetIsolator() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void TargetIsolator::Process(cv::Mat& source0, double altitude){
	//Step Blur0:
	//input
	cv::Mat blurInput = source0;
	double blurRadius = 1.8018018018018014;  // default Double
	blur(blurInput, blurRadius, this->blurOutput);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThreshold0Input = blurOutput;
	double hsvThreshold0Hue[] = {0.0, 180.0};
	double hsvThreshold0Saturation[] = {0.0, 38.531409168081495};
	double hsvThreshold0Value[] = {227.02338129496403, 255.0};
	hsvThreshold(hsvThreshold0Input, hsvThreshold0Hue, hsvThreshold0Saturation, hsvThreshold0Value, this->hsvThreshold0Output);
	//Step CV_dilate0:
	//input
	cv::Mat cvDilate0Src = hsvThreshold0Output;
	cv::Mat cvDilate0Kernel;
	cv::Point cvDilate0Anchor(-1, -1);
	double cvDilate0Iterations = 7.0;  // default Double
    int cvDilate0Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvDilate0Bordervalue(-1);
	cvDilate(cvDilate0Src, cvDilate0Kernel, cvDilate0Anchor, cvDilate0Iterations, cvDilate0Bordertype, cvDilate0Bordervalue, this->cvDilate0Output);
	//Step Mask0:
	//input
	cv::Mat maskInput = blurOutput;
	cv::Mat maskMask = cvDilate0Output;
	mask(maskInput, maskMask, this->maskOutput);
	//Step HSV_Threshold1:
	//input
	cv::Mat hsvThreshold1Input = maskOutput;
	double hsvThreshold1Hue[] = {155.39568345323738, 180.0};
	double hsvThreshold1Saturation[] = {98.60611510791367, 255.0};
	double hsvThreshold1Value[] = {91.72661870503596, 255.0};
	hsvThreshold(hsvThreshold1Input, hsvThreshold1Hue, hsvThreshold1Saturation, hsvThreshold1Value, this->hsvThreshold1Output);
	//Step CV_dilate1:
	//input
	cv::Mat cvDilate1Src = hsvThreshold1Output;
	cv::Mat cvDilate1Kernel;
	cv::Point cvDilate1Anchor(-1, -1);
	double cvDilate1Iterations = 2.0;  // default Double
    int cvDilate1Bordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvDilate1Bordervalue(-1);
	cvDilate(cvDilate1Src, cvDilate1Kernel, cvDilate1Anchor, cvDilate1Iterations, cvDilate1Bordertype, cvDilate1Bordervalue, this->cvDilate1Output);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = cvDilate1Output;
	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	//Step Filter_Contours0:
	//input
	std::vector<std::vector<cv::Point> > filterContoursContours = findContoursOutput;

	double focal_length = 1275.0; // perceived focal length of flir duo r in pixels, see https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
	double target_diameter = 36; //approx. 36 inches
	double margin = 0.5; // margin of error on either side

 	double filterContoursMinArea = 0.0;  // default Double
 	double filterContoursMinPerimeter = 0.0;  // default Double
 	double filterContoursMinWidth = ((target_diameter * focal_length) / altitude) * (1 - margin);
 	double filterContoursMaxWidth = ((target_diameter * focal_length) / altitude) * (1 + margin);
 	double filterContoursMaxHeight = filterContoursMaxWidth;
 	double filterContoursMinHeight = filterContoursMinWidth;
	double filterContoursSolidity[] = {0, 100};
	double filterContoursMaxVertices = 1000000.0;  // default Double
	double filterContoursMinVertices = 0.0;  // default Double
	double filterContoursMinRatio = 0.0;  // default Double
	double filterContoursMaxRatio = 1000.0;  // default Double
	filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->filterContoursOutput);
	//Step Convex_Hulls0:
	//input
	std::vector<std::vector<cv::Point> > convexHullsContours = filterContoursOutput;
	convexHulls(convexHullsContours, this->convexHullsOutput);
}

/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* TargetIsolator::GetBlurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* TargetIsolator::GetHsvThreshold0Output(){
	return &(this->hsvThreshold0Output);
}
/**
 * This method is a generated getter for the output of a CV_dilate.
 * @return Mat output from CV_dilate.
 */
cv::Mat* TargetIsolator::GetCvDilate0Output(){
	return &(this->cvDilate0Output);
}
/**
 * This method is a generated getter for the output of a Mask.
 * @return Mat output from Mask.
 */
cv::Mat* TargetIsolator::GetMaskOutput(){
	return &(this->maskOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* TargetIsolator::GetHsvThreshold1Output(){
	return &(this->hsvThreshold1Output);
}
/**
 * This method is a generated getter for the output of a CV_dilate.
 * @return Mat output from CV_dilate.
 */
cv::Mat* TargetIsolator::GetCvDilate1Output(){
	return &(this->cvDilate1Output);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* TargetIsolator::GetFindContoursOutput(){
	return &(this->findContoursOutput);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* TargetIsolator::GetFilterContoursOutput(){
	return &(this->filterContoursOutput);
}
/**
 * This method is a generated getter for the output of a Convex_Hulls.
 * @return ContoursReport output from Convex_Hulls.
 */
std::vector<std::vector<cv::Point> > TargetIsolator::GetConvexHullsOutput(){
	return this->convexHullsOutput;
}
	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void TargetIsolator::blur(cv::Mat &input, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize = 6 * radius + 1;
		cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
	}
		/**
		 * Filter out an area of an image using a binary mask.
		 *
		 * @param input The image on which the mask filters.
		 * @param mask The binary image that is used to filter.
		 * @param output The image in which to store the output.
		 */
		void TargetIsolator::mask(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
			mask.convertTo(mask, CV_8UC1);
			cv::bitwise_xor(output, output, output);
			input.copyTo(output, mask);
		}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void TargetIsolator::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void TargetIsolator::cvDilate(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void TargetIsolator::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters through contours.
	 * @param inputContours is the input vector of contours.
	 * @param minArea is the minimum area of a contour that will be kept.
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
	 * @param minWidth minimum width of a contour.
	 * @param maxWidth maximum width.
	 * @param minHeight minimum height.
	 * @param maxHeight  maximimum height.
	 * @param solidity the minimum and maximum solidity of a contour.
	 * @param minVertexCount minimum vertex Count of the contours.
	 * @param maxVertexCount maximum vertex Count.
	 * @param minRatio minimum ratio of width to height.
	 * @param maxRatio maximum ratio of width to height.
	 * @param output vector of filtered contours.
	 */
	void TargetIsolator::filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
		std::vector<cv::Point> hull;
		output.clear();
		for (std::vector<cv::Point> contour: inputContours) {
			cv::Rect bb = boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			double area = cv::contourArea(contour);
			if (area < minArea) continue;
			if (arcLength(contour, true) < minPerimeter) continue;
			cv::convexHull(cv::Mat(contour, true), hull);
			double solid = 100 * area / cv::contourArea(hull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
			double ratio = (double) bb.width / (double) bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.push_back(contour);
		}
	}

	/**
	 * Compute the convex hulls of contours.
	 *
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	void TargetIsolator::convexHulls(std::vector<std::vector<cv::Point> > &inputContours, std::vector<std::vector<cv::Point> > &outputContours) {
		std::vector<std::vector<cv::Point> > hull (inputContours.size());
		outputContours.clear();
		for (size_t i = 0; i < inputContours.size(); i++ ) {
			cv::convexHull(cv::Mat((inputContours)[i]), hull[i], false);
		}
		outputContours = hull;
	}



} // end grip namespace
