#include<ostream>
#include<stdio.h>
#include"opencv.hpp"
using namespace std;
class LaneDetector
{
private:
    double img_size;
    double img_center;
    bool left_flag = false;  // 是否检测到车道的左边界
    bool right_flag = false;  // 是否检测到车道的右边界
    cv::Point right_b;  // 车道边界的两个线方程的成员：
    double right_m;  // y = m*x + b直线方程
    cv::Point left_b;  //
    double left_m;  //

public:

    cv::Mat selectColor(cv::Mat inputImage); //图片的颜色选择
    cv::Mat deNoise(cv::Mat inputImage);  // 高斯滤波
    cv::Mat edgeDetector(cv::Mat img_noise);  // 边缘检测
    cv::Mat mask(cv::Mat img_edges);  // 获取ROI
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);  // 应用回归在车道的每一侧获得唯一的线
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);  // 通过斜率将线条检测到右侧和左侧线条
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);  // 车道的每一侧只能获得一条线
    std::string predictTurn();  // 通过计算消失点的位置来确定车道是否转弯
    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn);  // 绘制合成车道并在车架中转动预测
};
