#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/aruco.hpp>

class Image
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:

  int hmin, hmax;
  int smin, smax;
  int vmin, vmax;
  cv_bridge::CvImagePtr cv_ptr;

  Image() 
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &Image::imageCb, this);
    image_pub_ = it_.advertise("/detection/package2", 1);

    // Initialise Trackbars
    hmin = 15; hmax = 30;
    smin = 30; smax = 150;
    vmin = 30; vmax = 85;
  }

  ~Image()
  {
    cv::destroyAllWindows();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Original Image Window
    cv::imshow("Image", cv_ptr->image);

    // Create Trackbars
    // HSV_Trackbar();

    // Threshold in HSV
    Threshold();

    // Thresholded Image Window
    cv::imshow("Thresholded Image", cv_ptr->image);
    cv::waitKey(3);

    // Publish
    Publish();
  }

  void HSV_Trackbar()
  {
    cv::namedWindow("Trackbars",(500,500));

    // Trackbars
    cv::createTrackbar("H min", "Trackbars", &hmin, 179);
    cv::createTrackbar("H max", "Trackbars", &hmax, 179);
    cv::createTrackbar("S min", "Trackbars", &smin, 255);
    cv::createTrackbar("S max", "Trackbars", &smax, 255);
    cv::createTrackbar("V min", "Trackbars", &vmin, 255);
    cv::createTrackbar("V max", "Trackbars", &vmax, 255);
  }

  void Threshold ()
  {
    // cv::Mat imgBlur;
    cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(3,3),5,5);

    // cv::Mat imgHSV;
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV);

    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    cv::inRange(cv_ptr->image ,lower, upper, cv_ptr->image);
  }

  void Publish()
  {
    // Output modified video stream
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // message to be sent

    std_msgs::Header header; // empty header
    // header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, cv_ptr->image); // cv to cv_bridge
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    image_pub_.publish(img_msg);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "package_detector");
  Image im;
  ros::spin();
  return 0;
}
