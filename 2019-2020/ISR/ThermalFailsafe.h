//
// Created by zach on 2/22/19.
//
#pragma once
#ifndef CPP_UAV_THERMALFAILSAFE_H
#define CPP_UAV_THERMALFAILSAFE_H

#include "ISR.h"
#include "IRHuman.h"


class ThermalFailsafe {
private:
    grip::IRHuman *thermal_detector;
    void queue_in_db(std::vector<cv::Point>);

public:
    ThermalFailsafe();
    std::vector<cv::Point> process_img(cv::Mat img, double altitude);
};


#endif //CPP_UAV_THERMALFAILSAFE_H
