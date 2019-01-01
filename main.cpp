#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
//#include "LaneDetector.h"
#include "RoadSign.h"
#include "lanedetector.h"
int main() {

    std::string general_path="/home/jasmine/机器视觉/finalproject/finalproject/";
    std::string videoPath=general_path+"test_video/original.mp4";          //设置本地视频路径
    cv::VideoCapture cap(videoPath);            //读入视频
    if (!cap.isOpened())
        return -1;

    LaneDetector lanedetector;          // 创建车道检测类
    RoadSign roadsign;
    cv::Mat frame;
    cv::Mat selected_img;
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_wrape;
    cv::Mat M;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    std::string turn;
    int flag_plot = -1;
    int i = 0;

    //读取视频帧数
    double rate = cap.get(CV_CAP_PROP_FPS);
    // 获取视频帧的尺寸
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    //根据打开视频的参数初始化输出视频格式
    cv::VideoWriter lane_detor(general_path+"detect_out/lane_detected_orignal.mp4", CV_FOURCC('m', 'p', '4', 'v'), rate, cv::Size(width, height));
    // 对每一帧的视频进行处理
    while (i < 5500) {

        //读取视频
        if (!cap.read(frame))
            break;

        //选取颜色区域
        selected_img=lanedetector.selectColor(frame);
        roadsign.select_color(frame);

        // 高斯滤波
        img_denoise = lanedetector.deNoise(selected_img);

        // 边缘检测
        img_edges = lanedetector.edgeDetector(img_denoise);

        // 获得ROI(感兴趣区域）＋　滤波
        img_mask = lanedetector.mask(img_edges);
        img_mask=lanedetector.deNoise(img_mask);
        cv::Mat temp=img_mask.clone();

        //变换鸟瞰图
        M=lanedetector.get_M();
        img_wrape=lanedetector.perspective_trans(temp,M);
        lanedetector.find_line(img_wrape);
/*---------------------------------------------------------------------------------------*/
        // 在处理后的图像中获得霍夫线
        lines = lanedetector.houghLines(img_mask);

        if (!lines.empty())
        {
            // 检测车道是转向
            left_right_lines = lanedetector.lineSeparation(lines, img_edges);

            // 应用回归在车道的每一侧获得唯一的线
            lane = lanedetector.regression(left_right_lines, frame);

            // 通过确定线的消失点来预测转弯
            turn = lanedetector.predictTurn();

            // 在视频中显示方向
            flag_plot = lanedetector.plotLane(frame, lane, turn);
            lane_detor.write(frame);

            i += 1;
            cv::waitKey(25);
        }
        else {
            flag_plot = -1;
        }
    }
    return flag_plot;
}
