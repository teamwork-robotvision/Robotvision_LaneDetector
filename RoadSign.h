#include<ostream>
#include<stdio.h>
#include"opencv.hpp"
using namespace std;
using namespace cv;

class RoadSign{
private:
    Mat blue_mask;
    Mat green_mask;
    Mat yellow_mask;
public:
    //保留绿色，黄色，蓝色的路牌－－转换为HSV通道，颜色过滤
    void select_color(cv::Mat inputImage);

    //路标检测的感兴趣区域,避免过多的环境因素干扰
    Mat mask(Mat img);
};
