#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <pal_face_detector_opencv/cvRect.h>

class  FaceDepth
{
public:
     FaceDepth(ros::NodeHandle nh);
     ~FaceDepth();
     ros::Subscriber data_sub;
     image_transport::ImageTransport ImageTransport_;
     image_transport::Subscriber image_sub;
//     image_transport::Publisher img_pub;
protected:
    int x_rect, y_rect, height_rect, width_rect;
    cv::Mat img;
    cv_bridge::CvImage cvImg_;
    cv::Rect rectangle;

     void imageCB(const sensor_msgs::ImageConstPtr& msg);

     void dataCB(const pal_face_detector_opencv::cvRectConstPtr &data);
     void extractMat(cv::Mat orig, cv::Rect rect_);
//     void pubDepthImage(cv::Mat& img_);
     bool msg_check;
};

const std::string win1 = "Depth Image";
const std::string win2 = "Extracted Image";


FaceDepth::FaceDepth(ros::NodeHandle nh):ImageTransport_(nh)
{
    data_sub = nh.subscribe("pal_face/cv_rect_data", 1, &FaceDepth::dataCB, this);
    image_sub = ImageTransport_.subscribe("/xtion/depth/image_rect", 1, &FaceDepth::imageCB, this, image_transport::TransportHints("compressedDepth"));
    ROS_INFO("Subscribed to depth image");
//    img_pub = ImageTransport_.advertise("/pal_face/depth_image", 1);
    cv::namedWindow(win1, CV_WINDOW_FREERATIO);
    cv::namedWindow(win2, CV_WINDOW_FREERATIO);
    msg_check = false;
}

FaceDepth::~FaceDepth()
{
    cv::destroyAllWindows();
}

void FaceDepth::dataCB(const pal_face_detector_opencv::cvRectConstPtr& data)
{
    msg_check = true;
    x_rect = data->x * 2;
    y_rect = data->y * 2;
    height_rect = data->height*2;
    width_rect = data->width*2;

//    ROS_INFO_STREAM("IMG x,y" << x_rect << ", " << y_rect);
//    ROS_INFO_STREAM("IMG height, width" <<height_rect << ", " << width_rect);
}

void FaceDepth::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
//    ROS_INFO_STREAM("IMG height, width" << msg->height << ", " << msg->width);
    cv_bridge::CvImageConstPtr cvptr;
    cvptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cvptr->image.copyTo(img);
//    ROS_INFO_STREAM("SIZE: " << img.size);
//        ROS_INFO_STREAM("IMG rows, columns: " << img.rows << ", " << img.cols);

    cv::flip(img, img, 1);


//    cv::Rect rectangle;
    rectangle.x = x_rect;
    rectangle.y = y_rect;
    rectangle.height = height_rect;
    rectangle.width = width_rect;
    cv::rectangle(img, rectangle, CV_RGB(0,255,0));
    cv::imshow(win1, img);
    if(msg_check==true)
        this->extractMat(img, rectangle);

        //    this->pubDepthImage(img);

        cv::waitKey(1);

//    ROS_INFO_STREAM("Height, Width: " << h << ", " << w);
}


//void FaceDepth::pubDepthImage(cv::Mat& img_)
//{
//    cvImg_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//    sensor_msgs::Image imgMsg;
//    cvImg_.toImageMsg(imgMsg);
//    img_pub.publish(imgMsg);
//}

void FaceDepth::extractMat(cv::Mat orig, cv::Rect rect_)
{
    cv::Mat roi(orig, rect_);
    cv::imshow(win2, roi);
    ROS_INFO_STREAM("row, col: " << roi.rows << ", " << roi.cols);
    int total=0;
    int count;
    for(int i = 0; i<roi.rows; ++i)
        for(int j = 0; j<roi.cols; ++j)
        {
            auto depth = roi.at<short int>(cv::Point(i,j));
//            ROS_INFO_STREAM("depth: " << depth);
            total += depth;
            count++;
        }
    ROS_INFO_STREAM("average: " << total/count << " count: " << count);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_listener");

    ros::NodeHandle nh;

    FaceDepth fd(nh);

     ros::spin();

    return 0;
}
