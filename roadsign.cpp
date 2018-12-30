#include "RoadSign.h"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

cv::Mat RoadSign::select_color(cv::Mat inputImage){
    cv::Mat hsvImage;
    cv::Mat blue_mask;
    cv::Mat yellow_mask;
    cv::Mat green_mask;
    cv::cvtColor(inputImage,hsvImage,cv::COLOR_BGR2HSV);
    cv::inRange(hsvImage,cv::Scalar(100,50,50),cv::Scalar(124,255,255),blue_mask);
    cv::inRange(inputImage,cv::Scalar(0,160,120),cv::Scalar(0,255,140),green_mask);
    cv::inRange(inputImage,cv::Scalar(100,68,0),cv::Scalar(180,120,0),yellow_mask);
    cv::namedWindow("yellow",0);
    cv::resizeWindow("yellow",800,380);
    cv::namedWindow("blue",0);
    cv::resizeWindow("blue",800,380);
    cv::namedWindow("green",0);
    cv::resizeWindow("green",800,380);
    cv::imshow("yellow",yellow_mask);
    cv::imshow("blue",blue_mask);
    cv::imshow("green",green_mask);
    return blue_mask;


}
