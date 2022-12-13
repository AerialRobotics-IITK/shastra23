#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/aruco.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:

  int hmin, hmax;
  int smin, smax;
  int vmin, vmax;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/firefly/camera_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/package_detector/output", 1);

    cv::namedWindow(OPENCV_WINDOW);

    // // Trackbars
    // hmin = 0; hmax = 179;
    // smin = 0; smax = 255;
    // vmin = 0; vmax = 255;
    // cv::namedWindow("Trackbars",(500,500));
    // cv::createTrackbar("H min", "Trackbars", &hmin, 179);
    // cv::createTrackbar("H max", "Trackbars", &hmax, 179);
    // cv::createTrackbar("S min", "Trackbars", &smin, 255);
    // cv::createTrackbar("S max", "Trackbars", &smax, 255);
    // cv::createTrackbar("V min", "Trackbars", &vmin, 255);
    // cv::createTrackbar("V max", "Trackbars", &vmax, 255);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // // Blurring
    //     cv::GaussianBlur(cv_ptr->image,cv_ptr->image,cv::Size(3,3),5,5);
    
    // // HSV
    //     cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV);

    // // Threshold
    //     cv::Scalar lower(hmin, vmin, smin);
    //     cv::Scalar upper(hmax, vmax, smax);
    //     cv::inRange(cv_ptr->image,lower, upper, cv_ptr->image);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "package_detector");
  ImageConverter ic;
  ros::spin();
  return 0;
}