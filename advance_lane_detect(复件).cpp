//#include <string>
//#include <vector>
//#include <opencv2/opencv.hpp>
//#include <algorithm>
//#include<iterator>

//#include "lanedetector.h"//新的头文件－－匹配advance_lane_detect.cpp

///*代码改进：
// * 1.mask多边形的参数变化：粗略检测三个视频
// * 2.图像预处理不变，车道的检测使用新的拟合算法，不使用霍夫线拟合
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


//    //    cv::resize(img_noise,img_noise,cv::Size(),0.75,0.75);
//    // 灰度图
//    cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
//    // 二值化
//    cv::threshold(output, output, 140, 255, cv::THRESH_BINARY);

//    //    //创建内核[-1 0 1]
//    //    //这个内核基于在Mathworks的车道偏离警告系统
//    //    anchor = cv::Point(-1, -1);
//    //    kernel = cv::Mat(1, 3, CV_32F);
//    //    kernel.at<float>(0, 0) = -1;
//    //    kernel.at<float>(0, 1) = 0;
//    //    kernel.at<float>(0, 2) = 1;

//    //    // 过滤二进制图像以获得边缘
//    //    cv::filter2D(output, output, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);
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
//    cv::Point pts[4] = {
//        cv::Point(0, height),//左下角开始顺时针
//        cv::Point(width*0.3, height*0.65),
//        cv::Point(width*0.7, height*0.65),
//        cv::Point(width*0.9, height*0.95)
//    };//original的参数
//    //    cv::Point pts[4] = {
//    //        cv::Point(0, height*0.95),//左下角开始顺时针
//    //        cv::Point(width*0.1, height*0.75),
//    //        cv::Point(width*0.4, height*0.75),
//    //        cv::Point(width*0.6, height)
//    //    };//second01.02的参数
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

//    return output;
//}



///*--------------------------------------开始--------------------------*/
////定义方法获取变形矩阵-M和逆变形矩阵-Minv
//cv::Mat LaneDetector::get_M(){
//    cv::Mat M;
//    M=cv::getPerspectiveTransform(src,dst);
//    cv::namedWindow("M", 0);
//    cv::resizeWindow("M",800,380);
//    cv::imshow("M", M);
//    return M;

//}
//cv::Mat LaneDetector::get_Minv(){
//    cv::Mat Minv;
//    Minv=cv::getPerspectiveTransform(dst,src);
//    return Minv;
//}


////霍夫变换
//cv::Mat LaneDetector::perspective_trans(cv::Mat inputImage,cv::Mat M){
//    cv::Mat img_wraped;
//    //    int weight=inputImage.cols;
//    //    int height=inputImage.rows;
//    cv::warpPerspective(inputImage,img_wraped,M,inputImage.size());
//    cv::namedWindow("wraped", 0);
//    cv::resizeWindow("wraped",800,380);
//    cv::imshow("wraped", img_wraped);
//    return img_wraped;

//}



////拟合曲线
//cv::Mat LaneDetector:: polyfit(vector<cv::Point>& in_point, int n)
//{
//    int size = in_point.size();
//    //所求未知数个数
//    int x_num = n + 1;
//    //构造矩阵U和Y
//    cv::Mat mat_u(size, x_num, CV_64F);
//    cv::Mat mat_y(size, 1, CV_64F);

//    for (int i = 0; i < mat_u.rows; ++i)
//        for (int j = 0; j < mat_u.cols; ++j)
//        {
//            mat_u.at<double>(i, j) = pow(in_point[i].x, j);
//        }

//    for (int i = 0; i < mat_y.rows; ++i)
//    {
//        mat_y.at<double>(i, 0) = in_point[i].y;
//    }

//    //矩阵运算，获得系数矩阵K
//    cv::Mat mat_k(x_num, 1, CV_64F);
//    mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
//    cout << mat_k << endl;
//    return mat_k;
//}

////定位基点，滑动窗口多项式拟合－来获取车道边界这里使用9个200像素宽的滑动窗来定位一条车道线像素----//row-行;col--列；
//cv::Mat LaneDetector::find_line(cv::Mat inputImage){
//    //    //将鸟瞰图分半处理




//    cv::MatND histogram;

//    const int channels[1]={0};
//    const int histSize[1]={inputImage.cols};
//    float hranges[2]={0,255};
//    const float* ranges[1]={hranges};
//    cv::calcHist(&inputImage,1,channels,cv::Mat(),histogram,1,histSize,ranges);
//    //1280列.1920
//    //转置
//    cv::transpose(histogram,histogram);


//    //分左右车道
//    int midpoint=histogram.cols/2;//中点
//    int height=histogram.rows;

//    //将鸟瞰图的直方图分半处理
//    //右
//    cv::Mat  histogram_L=histogram(cv::Rect(0,0,midpoint,height));
//    double minL; double maxL;
//    cv::Point minLocL; cv::Point maxLocL;
//    minMaxLoc( histogram_L, &minL, &maxL, &minLocL, &maxLocL, cv::Mat() );

//    //左
//    cv::Mat  histogram_R=histogram(cv::Rect(midpoint,0,midpoint,height));
//    double minR; double maxR;
//    cv::Point minLocR; cv::Point maxLocR;
//    minMaxLoc( histogram_R, &minR, &maxR, &minLocR, &maxLocR, cv::Mat() );


//    int leftx_base=maxLocL.x;   //定义左基点
//    int rightx_base=maxLocR.x+midpoint;//定义右基点


//    int nwindows = 9;  //滑动窗口的个数

//    int window_height=inputImage.rows/nwindows;
//    //    cv::Mat  nonzero;//图像矩阵中非零元素location
//    vector<cv::Point> nonzero;
//    cv::findNonZero(inputImage,nonzero);//nonzero格式(a,b);(c,j)保存非零元素的位置

//    //Current positions to be updated for each window
//    int leftx_current = leftx_base;
//    int   rightx_current = rightx_base;

//    //Set the width of the windows +/- margin
//    int margin=100;
//    //Set minimum number of pixels found to recenter window
//    int minpix = 50;

//    //     Create empty lists to receive left and right lane pixel indices 保存滑动窗口中找到的点的集合
//    vector<int> left_lane_inds;
//    vector<int> right_lane_inds;

//    for(int window=0;window<nwindows;window++){

//        // 设置左右车道滑动窗口的边界x,y
//        int win_y_low=inputImage.rows-(window+1)*window_height;
//        int win_y_high = inputImage.rows - window*window_height;
//        int win_xleft_low = leftx_current - margin;
//        int win_xleft_high = leftx_current + margin;
//        int win_xright_low = rightx_current - margin;
//        int win_xright_high = rightx_current + margin;


//        //设置滑动窗口中的非零元素        # Identify the nonzero pixels in x and y within the window
//        //        左车道
//        vector<int>  good_left_inds;

//        for(int non_indL=0;non_indL<nonzero.size();non_indL++){

//            if((nonzero[non_indL].y>=win_y_low)&(nonzero[non_indL].y< win_y_high)&
//                    (nonzero[non_indL].x>= win_xleft_low)&(nonzero[non_indL].x<win_xleft_high)){

//                good_left_inds.push_back(non_indL);//将找到的车道的最佳点添加进去

//            }

//        }

//        //        右车道
//        vector<int>   good_right_inds;
//        for(int non_indR=0;non_indR<nonzero.size();non_indR++){

//            if((nonzero[non_indR].y>=win_y_low)&(nonzero[non_indR].y< win_y_high)&
//                    (nonzero[non_indR].x>= win_xright_low)&(nonzero[non_indR].x<win_xright_high)){

//                good_right_inds.push_back(non_indR);
//            }

//        }
//        //        # Append these indices to the lists

//        for(int i=0;i<good_left_inds.size();i++){
//            left_lane_inds.push_back(good_left_inds[i]);
//        }
//        for(int i=0;i<good_right_inds.size();i++){
//            right_lane_inds.push_back(good_right_inds[i]);
//        }
//        //        left_lane_inds.push_back(good_left_inds);
//        //        right_lane_inds.push_back(good_right_inds);



//        //        # If you found > minpix pixels, recenter next window on their mean position
//        if(good_left_inds.size()>minpix){
//            int sumL=0,meanL;
//            int index_tempL;
//            for(int i=0;i<good_left_inds.size();i++){
//                index_tempL=good_left_inds[i];
//                sumL=sumL+nonzero[index_tempL].x;
//            }
//            meanL=sumL/good_left_inds.size();
//            leftx_current=meanL;
//        }
//        if(good_right_inds.size()>minpix){
//            int sumR=0,meanR;
//            int index_tempR;
//            for(int i=0;i<good_left_inds.size();i++){
//                index_tempR=good_left_inds[i];
//                sumR=sumR+nonzero[index_tempR].x;
//            }
//            meanR=sumR/good_left_inds.size();
//        }
//        good_left_inds.clear();
//        good_right_inds.clear();
//    }

//    vector<cv::Point> left_line;
//    //对left_lane_inds和right_lane_inds中的点进行拟合
//    for(int i=0;i<left_lane_inds.size();i++){
//        int temp;
//        temp=left_lane_inds[i];
//        left_line.push_back(nonzero[temp]);
//    }
//    vector<cv::Point> right_line;
//    //对left_lane_inds和right_lane_inds中的点进行拟合
//    for(int i=0;i<right_lane_inds.size();i++){
//        int temp;
//        temp=right_lane_inds[i];
//        right_line.push_back(nonzero[temp]);
//    }

//    cv::Mat left_lane; //左车道的points
//    cv::Mat right_lane;//右车道的points
//    left_lane=polyfit(left_line,2);
//    right_lane=polyfit(right_line,2);

//    return histogram;

//}











///*------------------------------------结束---------------------------*/

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
//        output = "Left Turn";
//    else if (vanish_x >(img_center + thr_vp))
//        output = "Right Turn";
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
//    cv::putText(inputImage, turn, cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);

//    // 显示最终图片
//    cv::namedWindow("Lane", 0);
//    cv::resizeWindow("Lane",800,380);
//    cv::imshow("Lane", inputImage);
//    return 0;
//}
