#include"opencv.hpp"
using namespace cv;

class RoadSign{
private:
    Mat blue_mask;
    Mat green_mask;
    Mat yellow_mask;
    Mat red_mask;
public:
    //保留绿色，黄色，蓝色的路牌－－转换为HSV通道，颜色过滤
    void select_color(cv::Mat inputImage);
    void checkRoadSign(Mat inputImage,Mat outputImage,double colorLimit[],int binaryLimit[],int erodeNum[],int dilateNum[],int closeNum[],double angleLimit[],double wid_hei[],int sizeOfArea=5000,int someArea=0,int someOperate=0);

    //路标检测的感兴趣区域,避免过多的环境因素干扰
    Mat mask(Mat img);
};
