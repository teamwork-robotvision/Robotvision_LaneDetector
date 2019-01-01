#include<ostream>
#include<stdio.h>
#include"opencv.hpp"

//新的头文件－－匹配advance_lane_detect.cpp

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

    /*----------------改进添加------------------*/
    cv::Point2f src[4] = {
        cv::Point(203, 720),
        cv::Point(585, 460),
        cv::Point(695, 460),
        cv::Point(1127, 720)
    };
    cv::Point2f dst[4] = {
        cv::Point(320, 720),
        cv::Point(320, 0),
        cv::Point(960, 0),
        cv::Point(960, 720)
    };



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

    /*----------------------------------------开始变化---------------------------*/
    //新的拟合算法

    //定义方法获取变形矩阵-M和逆变形矩阵-Minv
    cv::Mat get_M();
    cv::Mat get_Minv();
    //霍夫变换
    cv::Mat perspective_trans(cv::Mat inputImage,cv::Mat M);

    //定位基点，滑动窗口多项式拟合－来获取车道边界这里使用9个200像素宽的滑动窗来定位一条车道线像素
    cv::Mat find_line(cv::Mat inputImage);
//    //查找一维矩阵的最大值下标
//    template<class ForwardIterator>
//    size_t argmax(ForwardIterator first, ForwardIterator last);
    cv::Mat polyfit(vector<cv::Point>& in_point, int n);
    cv::Mat calHist(cv::Mat inputImage);//计算直方图



};
