//#include <string>
//#include <vector>
//#include <opencv2/opencv.hpp>
//#include "LaneDetector.h"

///*代码改进：
// * 1.mask多边形的参数变化：粗略检测三个视频
// * 2.颜色选择
//*/


////图片的颜色选择

//cv::Mat LaneDetector::selectColor(cv::Mat inputImage){
//    cv::Mat white_mask;
//    cv::Mat yellow_mask;
//    cv::Mat mask_wy;
//    cv::Mat masked;
//    cv::inRange(inputImage,cv::Scalar(0,200,0),cv::Scalar(255,255,255),white_mask);
//    cv::inRange(inputImage,cv::Scalar(10,0,100),cv::Scalar(40,255,255),yellow_mask);
//    //    cv::namedWindow("white", 0);
//    //        cv::imshow("white",white_mask);
//    //    cv::namedWindow("yellow", 0);
//    //    cv::imshow("yellow",yellow_mask);

//    cv::bitwise_or(white_mask,yellow_mask,mask_wy);
//    //     cv::namedWindow("mask_wy", 0);
//    //     cv::imshow("mask_wy",mask_wy);

//    cv::bitwise_and(inputImage,inputImage,masked,mask_wy);
//    cv::namedWindow("masked", 0);
//    cv::resizeWindow("masked",800,380);
//    cv::imshow("masked", masked);
//    return masked;

//}



////高斯滤波

//cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
//    cv::Mat output;

//    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);

//    return output;
//}

//// 边缘检测
//cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
//    cv::Mat output;
//    cv::Mat kernel;
//    cv::Point anchor;



//    // 灰度图
//    cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
//    // 二值化
//    cv::threshold(output, output, 140, 255, cv::THRESH_BINARY);

//    cv::namedWindow("output", 0);
//    cv::resizeWindow("output",800,380);
//    cv::imshow("output", output);

//    return output;
//}

//// 边缘图像获取感兴趣区域
//cv::Mat LaneDetector::mask(cv::Mat img_edges) {
//    cv::Mat output;
//    int width,height;
//    width=img_edges.cols;
//    height=img_edges.rows;

//    cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
////    cv::Point pts[4] = {
////        cv::Point(0, height),//左下角开始顺时针
////        cv::Point(width*0.3, height*0.65),
////        cv::Point(width*0.7, height*0.65),
////        cv::Point(width*0.9, height*0.95)
////    };//original的参数
//                cv::Point pts[4] = {
//                    cv::Point(0, height*0.95),//左下角开始顺时针
//                    cv::Point(width*0.1, height*0.75),
//                    cv::Point(width*0.4, height*0.75),
//                    cv::Point(width*0.6, height)
//                };//second01.02的参数
//    //    cv::Point pts[4] = {
//    //        cv::Point(0, 720),
//    //        cv::Point(210, 450),
//    //        cv::Point(350, 450),
//    //        cv::Point(550, 720)
//    //    };

//    // 创建二进制多边形，填充多边形获取车道内部
//    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
//    cv::namedWindow("mask", 0);
//    cv::resizeWindow("mask",800,380);
//    cv::imshow("mask",mask);
//    // 将边缘图像和掩膜进行与操作以获得输出
//    cv::bitwise_and(img_edges, mask, output);

//    cv::namedWindow("ROI", 0);
//    cv::resizeWindow("ROI",800,380);
//    cv::imshow("ROI",output);

//    //    //查找多边形
//    //    cv::approxPolyDP(output,output,)
//    return output;
//}

//// 霍夫线
//std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
//    std::vector<cv::Vec4i> line;

//    // 使用概率Hough变换在二进制图像中查找线段
//    HoughLinesP(img_mask, line, 1, CV_PI / 180, 20, 20, 30);

//    return line;
//}

//// 筛选左右车道
//std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
//    std::vector<std::vector<cv::Vec4i> > output(2);
//    size_t j = 0;
//    cv::Point ini;
//    cv::Point fini;
//    double slope_thresh = 0.3;
//    std::vector<double> slopes;
//    std::vector<cv::Vec4i> selected_lines;
//    std::vector<cv::Vec4i> right_lines, left_lines;

//    // 计算所有检测到的线的斜率
//    for (auto i : lines) {
//        ini = cv::Point(i[0], i[1]);
//        fini = cv::Point(i[2], i[3]);

//        // slope = (y1 - y0)/(x1 - x0)－－斜率
//        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) / (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

//        // 如果斜率接近０，则丢弃该线
//        // 否则，分别保存斜率
//        if (std::abs(slope) > slope_thresh) {
//            slopes.push_back(slope);
//            selected_lines.push_back(i);
//        }
//    }

//    // 将线条分成右线和左线
//    img_center = static_cast<double>((img_edges.cols / 2));
//    while (j < selected_lines.size()) {
//        ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
//        fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

//        // 将线分类为左侧或右侧的条件
//        if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
//            right_lines.push_back(selected_lines[j]);
//            right_flag = true;
//        }
//        else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
//            left_lines.push_back(selected_lines[j]);
//            left_flag = true;
//        }
//        j++;
//    }

//    output[0] = right_lines;
//    output[1] = left_lines;

//    return output;
//}

//// 回归
//std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage) {
//    std::vector<cv::Point> output(4);
//    cv::Point ini;
//    cv::Point fini;
//    cv::Point ini2;
//    cv::Point fini2;
//    cv::Vec4d right_line;
//    cv::Vec4d left_line;
//    std::vector<cv::Point> right_pts;
//    std::vector<cv::Point> left_pts;

//    // 如果检测到右线，则使用线的所有初始点和最终点拟合线
//    if (right_flag == true) {
//        for (auto i : left_right_lines[0]) {
//            ini = cv::Point(i[0], i[1]);
//            fini = cv::Point(i[2], i[3]);

//            right_pts.push_back(ini);
//            right_pts.push_back(fini);
//        }

//        if (right_pts.size() > 0) {
//            // 合成右线
//            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
//            right_m = right_line[1] / right_line[0];
//            right_b = cv::Point(right_line[2], right_line[3]);
//        }
//    }

//    // 如果检测到左线，则使用线的所有初始点和最终点拟合线
//    if (left_flag == true) {
//        for (auto j : left_right_lines[1]) {
//            ini2 = cv::Point(j[0], j[1]);
//            fini2 = cv::Point(j[2], j[3]);

//            left_pts.push_back(ini2);
//            left_pts.push_back(fini2);
//        }

//        if (left_pts.size() > 0) {
//            // 合成左线
//            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
//            left_m = left_line[1] / left_line[0];
//            left_b = cv::Point(left_line[2], left_line[3]);
//        }
//    }

//    // 获得了一个斜率和偏移点，应用线方程来获得线点
//    int ini_y = inputImage.rows;
//    int fin_y = ini_y*0.8;

//    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
//    double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

//    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
//    double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

//    output[0] = cv::Point(right_ini_x, ini_y);
//    output[1] = cv::Point(right_fin_x, fin_y);
//    output[2] = cv::Point(left_ini_x, ini_y);
//    output[3] = cv::Point(left_fin_x, fin_y);

//    return output;
//}

//// 转向
//std::string LaneDetector::predictTurn() {
//    std::string output;
//    double vanish_x;
//    double thr_vp = 10;

//    // 消失点
//    vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

//    // 消失点位置决定了道路转弯的位置
//    if (vanish_x < (img_center - thr_vp))
//        output = "LEFT TURN";
//    else if (vanish_x >(img_center + thr_vp))
//        output = "RIGHT TURN";
//    else if (vanish_x >= (img_center - thr_vp) && vanish_x <= (img_center + thr_vp))
//        output = "Straight";

//    return output;
//}

//// 画出结果
//int LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn) {
//    std::vector<cv::Point> poly_points;
//    cv::Mat output;

//    // 创建清晰的多边形以更好地显示通道
//    inputImage.copyTo(output);
//    poly_points.push_back(lane[2]);
//    poly_points.push_back(lane[0]);
//    poly_points.push_back(lane[1]);
//    poly_points.push_back(lane[3]);
//    cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
//    cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);

//    // 绘制车道边界的两条线
//    cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, CV_AA);
//    cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, CV_AA);

//    // 显示转向
//    cv::putText(inputImage, turn, cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 3, cvScalar(0, 0, 0), 2, CV_AA);

//    // 显示最终图片
//    cv::namedWindow("Lane", 0);
//    cv::resizeWindow("Lane",800,380);
//    cv::imshow("Lane", inputImage);
//    return 0;
//}
