#include "RoadSign.h"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace cv;

void RoadSign::select_color(cv::Mat inputImage){
    vector<Mat> splitMat;
    double alpha=1.2; //1--3
    int beta=5; //0--100

    Mat imageSave=inputImage;
    for( int y = 0; y < inputImage.rows/3; y++ ) {
        for (int x = 0; x < inputImage.cols * 2 / 3; x++) {
            for (int c = 0; c < 3; c++) {
                inputImage.at<Vec3b>(y, x)[c] =
                        saturate_cast<uchar>(alpha * (inputImage.at<Vec3b>(y, x)[c]) + beta);
            }
        }
    }

    //滤波
    blur(inputImage,inputImage,Size(3,3));
    GaussianBlur(inputImage,inputImage,Size(3,3),0,0);
    medianBlur(inputImage,inputImage,5);

    //inputImage=mask(inputImage);

    /*****蓝色路标*****/
    //RGB转为HSV图像显示
    Mat hsvBlue;
    cv::cvtColor(inputImage,hsvBlue,cv::COLOR_BGR2HSV);

    //阈值分割
    cv::inRange(hsvBlue,cv::Scalar(100,50,50),cv::Scalar(124,255,255),blue_mask);
//    split(hsvBlue,splitMat);//分为3个通道
//    inRange(splitMat[0],Scalar(102,0.0,0,0),Scalar(124,0.0,0,0),splitMat[0]);
//    inRange(splitMat[1],Scalar(43.0,0.0,0,0),Scalar(255,0.0,0,0),splitMat[1]);
//    inRange(splitMat[2],Scalar(46,0.0,0,0),Scalar(255,0.0,0,0),splitMat[2]);
//
//    //按位与操作
//    bitwise_and(splitMat[2],splitMat[1],splitMat[1]);
//    bitwise_and(splitMat[0],splitMat[1],splitMat[0]);
//
//    blue_mask= splitMat[0].clone();
//    //merge(splitMat,blue_mask);

    //滤波
//    blur(blue_mask,blue_mask,Size(10,10));
//    GaussianBlur(blue_mask,blue_mask,Size(3,3),0,0);


    //创建窗口
    cv::namedWindow("blue",0);
    cv::resizeWindow("blue",800,380);

    //腐蚀膨胀
    erode(blue_mask,blue_mask,getStructuringElement(MORPH_RECT,Size(3,3)));
    dilate(blue_mask,blue_mask,getStructuringElement(MORPH_RECT,Size(5,5)));

    //二值化
    threshold(blue_mask,blue_mask,250,255,CV_THRESH_BINARY);

    //Canny(blue_mask,blue_mask,100,200);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(blue_mask,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    unsigned int index=0;
    while(index<contours.size()) {
        if (contours.at(index).size() >= 4&&hierarchy.at(index).val[2]>=0&&hierarchy.at(index).val[3]<0)
            drawContours(imageSave, contours, index, Scalar(255, 0, 0),2);
        index++;
    }

    //显示图像
    imshow("blue",imageSave);

    return;

    /*****绿色路标*****/
    //RGB转为HSV图像显示
    Mat hsvGreen;
    cv::cvtColor(inputImage,hsvGreen,cv::COLOR_BGR2HSV);

    //阈值分割
    split(hsvGreen,splitMat);//分为3个通道
    inRange(splitMat[0],Scalar(35,0.0,0,0),Scalar(77,0.0,0,0),splitMat[0]);
    inRange(splitMat[1],Scalar(43.0,0.0,0,0),Scalar(255,0.0,0,0),splitMat[1]);
    inRange(splitMat[2],Scalar(46,0.0,0,0),Scalar(255.0,0.0,0,0),splitMat[2]);

    //按位与操作
    bitwise_and(splitMat[2],splitMat[1],splitMat[1]);
    bitwise_and(splitMat[0],splitMat[1],splitMat[0]);
    //均值滤波
    blur(splitMat[0],splitMat[0],Size(3,3));

    green_mask= splitMat[0].clone();
    //merge(splitMat,green_mask);

    //均值滤波
    blur(green_mask,green_mask,Size(3,3));

    //创建窗口
    cv::namedWindow("green",0);
    cv::resizeWindow("green",800,380);

    //显示图像
    imshow("green",green_mask);

    /*****黄色路标*****/
    //RGB转为HSV图像显示
    Mat hsvYellow;
    cv::cvtColor(inputImage,hsvYellow,cv::COLOR_BGR2HSV);

    //阈值分割
    split(hsvYellow,splitMat);//分为3个通道
    inRange(splitMat[0],Scalar(26,0.0,0,0),Scalar(34,0.0,0,0),splitMat[0]);
    inRange(splitMat[1],Scalar(43.0,0.0,0,0),Scalar(255,0.0,0,0),splitMat[1]);
    inRange(splitMat[2],Scalar(46,0.0,0,0),Scalar(255.0,0.0,0,0),splitMat[2]);

    //按位与操作
    bitwise_and(splitMat[2],splitMat[1],splitMat[1]);
    bitwise_and(splitMat[0],splitMat[1],splitMat[0]);
    //均值滤波
    blur(splitMat[0],splitMat[0],Size(3,3));

    yellow_mask = splitMat[0].clone();
    //merge(splitMat,3,yellow_mask);

    //均值滤波
    blur(yellow_mask,yellow_mask,Size(3,3));

    //创建窗口
    cv::namedWindow("yellow",0);
    cv::resizeWindow("yellow",800,380);

    //显示图像
    imshow("yellow",yellow_mask);
}

Mat RoadSign::mask(cv::Mat img) {
    cv::Mat output;
    int width,height;
    width=img.cols;
    height=img.rows;

    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());

    //original的参数
    cv::Point pts[4] = {
            Point(width/7, height*4/5),//左下角开始顺时针
            Point(width/7, height/7),
            Point(width, height/7),
            Point(width, height*4/5)
    };

    // 创建二进制多边形，填充多边形获取感兴趣区域
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 255, 255));

    // 将边缘图像和掩膜进行与操作以获得输出
    cv::bitwise_and(img, mask, output);

    cv::namedWindow("ROI", 0);
    cv::resizeWindow("ROI",800,380);
    cv::imshow("ROI",output);

    return output;
}
