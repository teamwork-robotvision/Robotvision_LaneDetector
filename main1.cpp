#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>
#include "LaneDetector.h"
#include "RoadSign.h"

using namespace std;
using namespace cv;
//#include "lanedetector.h"

int main() {

    //std::string general_path="/home/jasmine/机器视觉/finalproject/finalproject/";
    string general_path="/home/zhaobin/opencv/Robotvision_LaneDetector/";
    std::string videoPath=general_path+"test_video/second01.mp4";          //设置本地视频路径
    cv::VideoCapture cap(videoPath);            //读入视频
    if (!cap.isOpened()){
        cout<<"can not open the video"<<endl;
        return -1;
    }

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

    //红色路标检测参数
    double colorLimit_red[]={156,43,46,180,255,255};
    int binaryLimit_red[]={235,255};
    int erodeNum_red[]={3,3};
    int dilateNum_red[]={5,5};
    int closeNum_red[]={10,10};
    double angleLimit_red[]={-90/CV_PI,90/CV_PI};
    double wid_hei_red[]={1,5};
    int sizeOfArea_red=5000;

    //黄色路标检测参数
    double colorLimit_yellow[]={20,43,46,24,255,255};
    int binaryLimit_yellow[]={220,255};
    int erodeNum_yellow[]={3,3};
    int dilateNum_yellow[]={5,5};
    int closeNum_yellow[]={3,3};
    double angleLimit_yellow[]={-50/CV_PI,50/CV_PI};
    double wid_hei_yellow[]={0.9,3.1};
    int sizeOfArea_yellow=3500;

    //蓝色路标检测参数
    double colorLimit_blue[]={102.5,43,46,124,255,255};
    int binaryLimit_blue[]={235,255};
    int erodeNum_blue[]={3,3};
    int dilateNum_blue[]={7,7};
    int closeNum_blue[]={10,10};
    double angleLimit_blue[]={-90/CV_PI,90/CV_PI};
    double wid_hei_blue[]={0.91,4.5};
    int sizeOfArea_blue=50000;

    //绿色路标检测参数
    double colorLimit_green[]={96,43,46,99,255,255};
    int binaryLimit_green[]={235,255};
    int erodeNum_green[]={3,3};
    int dilateNum_green[]={7,7};
    int closeNum_green[]={10,10};
    double angleLimit_green[]={-30/CV_PI,30/CV_PI};
    double wid_hei_green[]={1,1};
    int sizeOfArea_green=3500;

    // 对每一帧的视频进行处理
    while (1){

        //读取视频
        if (!cap.read(frame))
            break;
        roadsign.checkRoadSign(frame,frame,colorLimit_red,binaryLimit_red,erodeNum_red,dilateNum_red,closeNum_red,angleLimit_red,wid_hei_red,sizeOfArea_red);
        roadsign.checkRoadSign(frame,frame,colorLimit_yellow,binaryLimit_yellow,erodeNum_yellow,dilateNum_yellow,closeNum_yellow,angleLimit_yellow,wid_hei_yellow,sizeOfArea_yellow);
        roadsign.checkRoadSign(frame,frame,colorLimit_blue,binaryLimit_blue,erodeNum_blue,dilateNum_blue,closeNum_blue,angleLimit_blue,wid_hei_blue,sizeOfArea_blue);
        roadsign.checkRoadSign(frame,frame,colorLimit_green,binaryLimit_green,erodeNum_green,dilateNum_green,closeNum_green,angleLimit_green,wid_hei_green,sizeOfArea_green);
        if(waitKey(10)>0)
            waitKey(0);

        namedWindow("frame",0);
        imshow("frame",frame);
    }

    return flag_plot;
}
