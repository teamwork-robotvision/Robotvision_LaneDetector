#include<ostream>
#include<stdio.h>
#include"opencv.hpp"
using namespace std;

class RoadSign{
private:

public:
    //    保留绿色，黄色，蓝色的路牌－－转换为HSV通道，颜色过滤
    cv::Mat select_color(cv::Mat inputImage);


};
