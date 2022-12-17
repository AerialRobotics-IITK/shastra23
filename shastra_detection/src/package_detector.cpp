#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/aruco.hpp>

class Image
{
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
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
    point_cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &Image::point_cloud_cb, this);

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

    // Boundaries and Center
    Contours();

    // Thresholded Image Window
    cv::imshow("Thresholded Image", cv_ptr->image);
    cv::waitKey(3);

    // Publish
    Publish();
  }

  void point_cloud_cb (const sensor_msgs::PointCloud2 pCloud)
  {
    geometry_msgs::Point pkg_position_camera_frame;
    pixel_to_3d_point(pCloud, center.x, center.y, pkg_position_camera_frame);
  }

  void pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
  {
    // get width and height of 2D point cloud data;
    int width = pCloud.width;
    int height = pCloud.height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X, Y, Z data starts
    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

    // compute position in array where x, y, z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
    
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;
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

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::erode(cv_ptr->image, cv_ptr->image, kernel);
  }

  void Contours()
  {
    // Canny
    // cv::Canny(cv_ptr->image, cv_ptr->image,50,50);

    // Contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;
    cv::findContours(cv_ptr->image, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Filtering
    int area;
    float peri;
    std::vector<std::vector<cv::Point>> conPoly( contours.size() );
    std::vector<cv::Rect> boundRect(contours.size());

    for (int i = 0; i < contours.size(); i++)
    {
        area = cv::contourArea(contours[i]);
        // std::cout << area << std::endl;

        if (area > 25)
        {
            // Number of corner points
            cv::approxPolyDP(contours[i],conPoly[i] ,0.02 * peri, 0);
            
            // Bounding Rectangle
            boundRect[i] = cv::boundingRect(conPoly[i]);

            // One issue here is that it is not exact center due to 3D veiw
            cv::Point center(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);

            // Covert to 3d
        }
    }
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
