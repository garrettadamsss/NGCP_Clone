#include "IRHuman.h"

namespace grip {

    IRHuman::IRHuman() {
    }

/**
* Runs an iteration of the pipeline and updates outputs.
*/
    void IRHuman::Process(cv::Mat &source0, double altitude) {
        //Step Blur0:
        //input
        cv::Mat blurInput = source0;
        double blurRadius = 0.0;  // default Double
        blur(blurInput, blurRadius, this->blurOutput);
        //Step HSV_Threshold0:
        //input
        cv::Mat hsvThresholdInput = blurOutput;
        double hsvThresholdHue[] = {0.0, 180.0};
        double hsvThresholdSaturation[] = {0.0, 255.0};
        double hsvThresholdValue[] = {222.43705035971226, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue,
                     this->hsvThresholdOutput);
        //Step Find_Contours0:
        //input
        cv::Mat findContoursInput = hsvThresholdOutput;
        bool findContoursExternalOnly = false;  // default Boolean
        findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
        //Step Convex_Hulls0:
        //input
        std::vector<std::vector<cv::Point> > convexHullsContours = findContoursOutput;
        convexHulls(convexHullsContours, this->convexHullsOutput);
    }

/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
    cv::Mat *IRHuman::GetBlurOutput() {
        return &(this->blurOutput);
    }

/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
    cv::Mat *IRHuman::GetHsvThresholdOutput() {
        return &(this->hsvThresholdOutput);
    }

/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
    std::vector<std::vector<cv::Point> > *IRHuman::GetFindContoursOutput() {
        return &(this->findContoursOutput);
    }

/**
 * This method is a generated getter for the output of a Convex_Hulls.
 * @return ContoursReport output from Convex_Hulls.
 */
    std::vector<std::vector<cv::Point> > IRHuman::GetConvexHullsOutput() {
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
    void IRHuman::blur(cv::Mat &input, double doubleRadius, cv::Mat &output) {
        int radius = (int) (doubleRadius + 0.5);
        int kernelSize;
        kernelSize = 2 * radius + 1;
        cv::blur(input, output, cv::Size(kernelSize, kernelSize));
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
    void IRHuman::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
        cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
        cv::inRange(out, cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Finds contours in an image.
     *
     * @param input The image to find contours in.
     * @param externalOnly if only external contours are to be found.
     * @param contours vector of contours to put contours in.
     */
    void IRHuman::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
        std::vector<cv::Vec4i> hierarchy;
        contours.clear();
        int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
        int method = cv::CHAIN_APPROX_SIMPLE;
        cv::findContours(input, contours, hierarchy, mode, method);
    }

    /**
     * Compute the convex hulls of contours.
     *
     * @param inputContours The contours on which to perform the operation.
     * @param outputContours The contours where the output will be stored.
     */
    void IRHuman::convexHulls(std::vector<std::vector<cv::Point> > &inputContours,
                              std::vector<std::vector<cv::Point> > &outputContours) {
        std::vector<std::vector<cv::Point> > hull(inputContours.size());
        outputContours.clear();
        for (size_t i = 0; i < inputContours.size(); i++) {
            cv::convexHull(cv::Mat((inputContours)[i]), hull[i], false);
        }
        outputContours = hull;
    }


} // end grip namespace

