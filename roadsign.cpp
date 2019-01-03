#include "RoadSign.h"
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
#define BLUE 3
#define GREEN 3
#define YELLOW 0

//路标检测
void RoadSign::select_color(cv::Mat inputImage) {
    Mat hsv;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat imageSave = inputImage.clone();

    //增加对比度和饱和度
//    double alpha = 1.2; //1--3
//    int beta = 5; //0--100
//    for( int y = 0; y < inputImage.rows/3; y++ ) {
//        for (int x = 0; x < inputImage.cols * 2 / 3; x++) {
//            for (int c = 0; c < 3; c++) {
//                inputImage.at<Vec3b>(y, x)[c] =
//                        saturate_cast<uchar>(alpha * (inputImage.at<Vec3b>(y, x)[c]) + beta);
//            }
//        }
//    }

    //滤波
    blur(inputImage, inputImage, Size(3, 3));
    GaussianBlur(inputImage, inputImage, Size(3, 3), 0, 0);
    medianBlur(inputImage, inputImage, 5);

    //RGB转为HSV图像显示
    cv::cvtColor(inputImage, hsv, cv::COLOR_BGR2HSV);

//    /*******************************************蓝色路标开始*******************************************/
//    //阈值分割
//    //cv::inRange(hsv, cv::Scalar(214-BLUE,61-BLUE, 62-BLUE), cv::Scalar(214+BLUE, 61+BLUE,62+BLUE), blue_mask);
//    cv::inRange(hsv, cv::Scalar(100 + BLUE, 50 + BLUE, 50 + BLUE), cv::Scalar(124 - BLUE, 255 - BLUE, 255 - BLUE),
//                blue_mask);
//
//    //二值化
//    threshold(blue_mask, blue_mask, 220, 255, CV_THRESH_BINARY);
//
//    namedWindow("binary",0);
//    imshow("binary", blue_mask);
//
//    //腐蚀膨胀
//    erode(blue_mask, blue_mask, getStructuringElement(MORPH_RECT, Size(3, 3)));
//
//    namedWindow("erode",0);
//    imshow("erode", blue_mask);
//
//    dilate(blue_mask, blue_mask, getStructuringElement(MORPH_RECT, Size(3, 3)));
//
//    namedWindow("dilate",0);
//    imshow("dilate", blue_mask);
//
//    //闭运算
//    Mat kernel = getStructuringElement(MORPH_RECT, Size(10,10));
//    morphologyEx(blue_mask, blue_mask, MORPH_CLOSE, kernel);
//
//    namedWindow("close",0);
//    imshow("close", blue_mask);
//
//
//    //创建窗口
//    cv::namedWindow("blue", 0);
//    cv::resizeWindow("blue", 800, 380);
//
//    /*********************************添加感兴趣区域***************************************/
//    int width, height;
//    width = blue_mask.cols;
//    height = blue_mask.rows;
//
//    //original的参数
//    cv::Point pts[4] = {
//            Point(width / 5, height / 4),//左下角开始顺时针
//            Point(width /5 , 0),
//            Point(width * 4 / 5, 0),
//            Point(width * 4 / 5, height / 4)
//    };
//
//    // 创建二进制多边形，填充多边形获取感兴趣区域
//    cv::fillConvexPoly(blue_mask, pts, 4, cv::Scalar(0,0,0));
//    imshow("blue", blue_mask);
//    /*************************************************************/
//
//
//    //查找轮廓并显示
//    findContours(blue_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
//    unsigned int index = 0;
//    while (index < contours.size()) {
//        if (contours.at(index).size() >= 4 && hierarchy.at(index).val[2] >= 0 && hierarchy.at(index).val[3] < 0) {
//            RotatedRect cnt = minAreaRect(contours.at(index));
//            if (cnt.angle > -90/CV_PI&& cnt.angle < 90/CV_PI) {
//                int middle = cnt.size.width / cnt.size.height;
//                if(middle == 1|| (middle > 2&&middle <= 5) && cnt.size.area() < 50000) {
//                    //drawContours(imageSave, contours, index, Scalar(0, 0, 255), 2);
//                    Point2f box[4];
//                    cnt.points(box);
//                    rectangle(imageSave,box[0],box[2],Scalar(0,255,255),2);
////                    vector<vector<Point>> rec;
////                    vector<Point> v;
////                    for(int i=0;i<4;i++) {
////                        v.push_back(box[i]);
////                    }
////                    rec.push_back(v);
////                    drawContours(imageSave, rec, 0, Scalar(0, 0, 255), 2);
//                }
//            }
//        }
//        index++;
//    }
//
//    /*******************************************蓝色路标结束*******************************************/
//
//    /*******************************************绿色路标开始*******************************************/
//    //阈值分割
//    //cv::inRange(hsv, cv::Scalar(172 - GREEN, 100 - GREEN, 57 - GREEN), cv::Scalar(172 + GREEN, 100 + GREEN, 57 + GREEN),
//    //            green_mask);
//    cv::inRange(hsv, cv::Scalar(35 + GREEN, 43 + GREEN, 46 + GREEN), cv::Scalar(77 - GREEN, 255 - GREEN, 255 - GREEN),
//                green_mask);
//
//    //创建窗口
//    cv::namedWindow("green", 0);
//    cv::resizeWindow("green", 800, 380);
//
//    findContours(green_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
//
//    //显示图像
//    imshow("green", green_mask);
//
//    /*******************************************绿色路标结束*******************************************/


    /*******************************************黄色路标开始*******************************************/
    //阈值分割
    //cv::inRange(hsv,cv::Scalar(11-YELLOW, 43-YELLOW,46-YELLOW), cv::Scalar(34+YELLOW,255+YELLOW,255+YELLOW), yellow_mask);
    //cv::inRange(hsv, cv::Scalar(11 + YELLOW, 43 + YELLOW, 46 + YELLOW),cv::Scalar(34 - YELLOW, 255 - YELLOW, 255 - YELLOW), yellow_mask);
    cv::inRange(hsv, cv::Scalar(26, 43 , 46),cv::Scalar(34, 255, 255), yellow_mask);

    blur(yellow_mask,yellow_mask,Size(5,5));

    //二值化
    threshold(yellow_mask, yellow_mask, 235, 255, CV_THRESH_BINARY);

    namedWindow("binary_yellow",0);
    imshow("binary_yellow", yellow_mask);

    //腐蚀膨胀
    erode(yellow_mask, yellow_mask, getStructuringElement(MORPH_RECT, Size(3, 3)));

    namedWindow("erode_yellow",0);
    imshow("erode_yellow", yellow_mask);

    dilate(yellow_mask, yellow_mask, getStructuringElement(MORPH_RECT, Size(5,5)));

    namedWindow("dilate_yellow",0);
    imshow("dilate_yellow", yellow_mask);

    //闭运算
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));
    morphologyEx(yellow_mask, yellow_mask, MORPH_CLOSE, kernel);

    namedWindow("close_yellow",0);
    imshow("close_yellow", yellow_mask);


    //创建窗口
    cv::namedWindow("yellow", 0);
    cv::resizeWindow("yellow", 800, 380);

    imshow("yellow",yellow_mask);

    //查找轮廓并显示
    findContours(yellow_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    unsigned int index = 0;
    while (index < contours.size()) {
        if (contours.at(index).size() >= 4 && hierarchy.at(index).val[2] >= 0 && hierarchy.at(index).val[3] < 0) {
            RotatedRect cnt = minAreaRect(contours.at(index));
            if (cnt.angle > -90 / CV_PI && cnt.angle < 90 / CV_PI) {
                int middle = cnt.size.width / cnt.size.height;
                if (middle == 1 || (middle > 2 && middle <= 5) && cnt.size.area() < 750) {
                    //drawContours(imageSave, contours, index, Scalar(0, 0, 255), 2);
                    Point2f box[4];
                    cnt.points(box);
                    rectangle(imageSave, box[0], box[2], Scalar(0, 255, 255), 2);
                }
            }
        }
        index++;
    }

    /*******************************************黄色路标结束*******************************************/

    /*******************************************红色路标开始*******************************************/
    //阈值分割
    cv::inRange(hsv, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255),green_mask);

    //二值化
    threshold(red_mask, red_mask, 220, 255, CV_THRESH_BINARY);

    namedWindow("binary_red",0);
    imshow("binary_red", red_mask);

    //腐蚀膨胀
    erode(red_mask, red_mask, getStructuringElement(MORPH_RECT, Size(3, 3)));

    namedWindow("erode_red",0);
    imshow("erode_red", red_mask);

    dilate(red_mask, red_mask, getStructuringElement(MORPH_RECT, Size(3, 3)));

    namedWindow("dilate_red",0);
    imshow("dilate_red", red_mask);

    //闭运算
    kernel = getStructuringElement(MORPH_RECT, Size(10,10));
    morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);

    namedWindow("close_red",0);
    imshow("close_red", red_mask);


    //创建窗口
    cv::namedWindow("red", 0);
    cv::resizeWindow("red", 800, 380);

    imshow("red",red_mask);

    //查找轮廓并显示
    findContours(red_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    index = 0;
    while (index < contours.size()) {
        if (contours.at(index).size() >= 4 && hierarchy.at(index).val[2] >= 0 && hierarchy.at(index).val[3] < 0) {
            RotatedRect cnt = minAreaRect(contours.at(index));
            if (cnt.angle > -90 / CV_PI && cnt.angle < 90 / CV_PI) {
                int middle = cnt.size.width / cnt.size.height;
                if (middle == 1 || (middle > 2 && middle <= 5) && cnt.size.area() < 750) {
                    //drawContours(imageSave, contours, index, Scalar(0, 0, 255), 2);
                    Point2f box[4];
                    cnt.points(box);
                    rectangle(imageSave, box[0], box[2], Scalar(0, 255, 255), 2);
                }
            }
        }
        index++;
    }


    /*******************************************红色路标结束*******************************************/

    namedWindow("output",0);
    imshow("output",imageSave);
}

//合并的路标检测函数
void RoadSign::checkRoadSign(Mat inputImage,Mat outputImage,double colorLimit[],int binaryLimit[],int erodeNum[],int dilateNum[],int closeNum[],double angleLimit[],double wid_hei[],int sizeOfArea,int someArea,int someOperate){
    Mat hsv,operateMat;;
    Mat operateImage = inputImage.clone();

    //增加对比度和饱和度
//    double alpha = 1.2; //1--3
//    int beta = 5; //0--100
//    for( int y = 0; y < operateImage.rows/3; y++ ) {
//        for (int x = 0; x < operateImage.cols * 2 / 3; x++) {
//            for (int c = 0; c < 3; c++) {
//                operateImage.at<Vec3b>(y, x)[c] =
//                        saturate_cast<uchar>(alpha * (operateImage.at<Vec3b>(y, x)[c]) + beta);
//            }
//        }
//    }

    //滤波
    if(someOperate) {
        blur(operateImage, operateImage, Size(3, 3));
        GaussianBlur(operateImage, operateImage, Size(3, 3), 0, 0);
        medianBlur(operateImage, operateImage, 5);
    }

    //RGB转为HSV图像显示
    cv::cvtColor(operateImage, hsv, cv::COLOR_BGR2HSV);

    //阈值分割
    //cv::inRange(hsv, cv::Scalar(214-BLUE,61-BLUE, 62-BLUE), cv::Scalar(214+BLUE, 61+BLUE,62+BLUE), blue_mask);
    cv::inRange(hsv, cv::Scalar(colorLimit[0], colorLimit[1],colorLimit[2]), cv::Scalar(colorLimit[3],colorLimit[4],colorLimit[5]),
                operateMat);

    //二值化
    threshold(operateMat, operateMat, binaryLimit[0],binaryLimit[1], CV_THRESH_BINARY);

    namedWindow("binary",0);
    imshow("binary", operateMat);

    //腐蚀膨胀
    erode(operateMat, operateMat, getStructuringElement(MORPH_RECT, Size(erodeNum[0],erodeNum[1])));

    namedWindow("erode",0);
    imshow("erode", operateMat);

    dilate(operateMat, operateMat, getStructuringElement(MORPH_RECT, Size(dilateNum[0],dilateNum[1])));

    namedWindow("dilate",0);
    imshow("dilate", operateMat);

    //闭运算
    Mat kernel = getStructuringElement(MORPH_RECT, Size(closeNum[0],closeNum[1]));
    morphologyEx(operateMat, operateMat, MORPH_CLOSE, kernel);

    namedWindow("close",0);
    imshow("close", operateMat);

    /***************************添加感兴趣区域*****************/
    if(someArea) {
        int width, height;
        width = operateMat.cols;
        height = operateMat.rows;

        //original的参数
        cv::Point pts[4] = {
                Point(width / 5, height / 4),//左下角开始顺时针
                Point(width / 5, 0),
                Point(width * 4 / 5, 0),
                Point(width * 4 / 5, height / 4)
        };

        // 创建二进制多边形，填充多边形获取感兴趣区域
        cv::fillConvexPoly(operateMat, pts, 4, cv::Scalar(0, 0, 0));
        imshow("blue", operateMat);
    }
    /*************************************************************/

    //查找轮廓并显示
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(operateMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> approxContours(contours.size());
    unsigned int index = 0;
    while (index < contours.size()){
        approxPolyDP(contours[index],approxContours[index],20,true);
        if(approxContours.at(index).size()>15){
            drawContours(outputImage,approxContours,index,Scalar(0,0,255),2);
            index++;
            continue;
        }

        //轮廓筛选
        if (contours.at(index).size() >= 4 /*轮廓角点初步筛选*/
        && hierarchy.at(index).val[2] >= 0 && hierarchy.at(index).val[3] < 0 /*找具有内轮廓的最外层轮廓*/) {
            RotatedRect cnt = minAreaRect(contours.at(index));//找出外接矩阵
            if (cnt.angle > angleLimit[0]&& cnt.angle <=angleLimit[1]) {//矩阵外界角度筛选
                int middle = cnt.size.width / cnt.size.height;
                if(middle >= wid_hei[0]&&middle <= wid_hei[1]&& cnt.size.area() < sizeOfArea) {//矩阵长宽比和矩阵大小筛选
                    Point2f box[4];
                    cnt.points(box);
                    //画出外接矩阵
                    rectangle(outputImage,box[0],box[2],Scalar(0,255,255),2);
//                    vector<vector<Point>> rec;
//                    vector<Point> v;
//                    for(int i=0;i<4;i++) {
//                        v.push_back(box[i]);
//                    }
//                    rec.push_back(v);
//                    drawContours(imageSave, rec, 0, Scalar(0, 0, 255), 2);
                }
            }
        }
        index++;
    }
}

//获取感兴趣区域
Mat RoadSign::mask(cv::Mat img) {
    cv::Mat output;
    int width, height;
    width = img.cols;
    height = img.rows;

    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());

    //original的参数
//    cv::Point pts[4] = {
//            Point(width / 5, height / 4),//左下角开始顺时针
//            Point(width /5 , 0),
//            Point(width * 4 / 5, 0),
//            Point(width * 4 / 5, height / 4)
//    };

    cv::Point pts[4] = {
            Point(width * 2 / 5, height / 5),//左下角开始顺时针
            Point(width * 2 /5 , 0),
            Point(width * 4 / 5, 0),
            Point(width * 4 / 5, height / 5)
    };

    // 创建二进制多边形，填充多边形获取感兴趣区域
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 255, 255));

    // 将边缘图像和掩膜进行与操作以获得输出
    cv::bitwise_and(img, mask, output);

    cv::namedWindow("ROI", 0);
    cv::resizeWindow("ROI", 800, 380);
    cv::imshow("ROI", output);

    return output;
}
