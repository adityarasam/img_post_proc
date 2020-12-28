#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/types.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/core/fast_math.hpp>

static const std::string OPENCV_WINDOW = "Camera Image Original";
static const std::string OPENCV_EDGE = "Camera Image EDGE";
static const std::string OPENCV_WINDOW_FAST = "Camera Image Original FAST";
static const std::string OPENCV_WINDOW_HIST_EQUALIZED = "Image_HE window";
static const std::string OPENCV_WINDOW_HIST_EQUALIZED_FAST = "Camera Image CLAHE FAST";
static const std::string OPENCV_WINDOW_CLAHE = "Camera Image CLAHE"; 
int lowThreshold = 1;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 7;

class ImagebridgeRosCV
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:

  int feature_type;                                           // 0 FAST, 1 SURF, 2 SIFT, ORB  
  ImagebridgeRosCV()
    : it_(nh_), feature_type(0)
  {
    // Subscrive to input video feed and publish output video feed
    ros::NodeHandle pnh;
    pnh.getParam("/image_converter_post_proc/feature_type", feature_type);
    pnh.param("/image_converter_post_proc/feature_type", feature_type, 2);

    ROS_INFO("  feature_type=%d", feature_type);

    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImagebridgeRosCV::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/image_raw", 1);

    //cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(10, true,  cv::FastFeatureDetector::TYPE_9_16);
    
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_EDGE);
    cv::namedWindow(OPENCV_WINDOW_HIST_EQUALIZED);
    cv::namedWindow(OPENCV_WINDOW_FAST);
    cv::namedWindow(OPENCV_WINDOW_HIST_EQUALIZED_FAST);
    cv::namedWindow(OPENCV_WINDOW_CLAHE);

    if (feature_type == 0){
        std::cout<<"Feature Type: FAST \n";
    }
    else if (feature_type == 1)
    {
        std::cout<<"Feature Type: SURF \n";
    }
    else if (feature_type == 2){
        std::cout<<"Feature Type: SIFT \n";
    }

  }
  
  ~ImagebridgeRosCV()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImage res_cv;
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(25, true,  cv::FastFeatureDetector::TYPE_9_16);
    cv::Ptr<cv::CLAHE> obj_CLAHE = cv::createCLAHE(2, cv::Size(4,4));

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);  //BGR8
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::Mat flipped_image,hist_equalized_image,CLAHE_image, flipped_image_hist_calc, CLAHE_image_hist_calc;
    cv::Mat flipped_image_with_kp, hist_equalized_image_with_kp;
    cv::Mat CLAHE_image_smoothed, flipped_image_smooth; 
    cv::Mat edge_detected;
    std::vector<cv::KeyPoint> keypoints_orig, keypoints_hist_equa;

    // Rotate image 180 degree
    //cv::flip(cv_ptr->image, flipped_image, -1);
    cv::flip(cv_ptr->image, flipped_image, 1);                                                         // ------------Flip image
    cv::GaussianBlur(flipped_image, flipped_image_smooth, cv::Size(5,5), 1);                                 
    //cv::bilateralFilter(flipped_image, flipped_image_smooth, 5,  2, 2);                                 // ------------Smooth image but retain edginess
    // Plot the histogram
    //?????
    //flipped_image = flipped_image_smooth;
    flipped_image_hist_calc = flipped_image;

    //------------------------------histogram plotting
    std::vector<cv::Mat> g_plane, g_plane_CLAHE;
    cv::Mat g_hist;
    int histSize = 256;
    float range[] = { 0, 256 }; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    //! [Draw the histograms for B, G and R]
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );




    //-----------Calculate and Plot Orig image HIST
    split( flipped_image_hist_calc, g_plane );
    calcHist( &g_plane[0], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 235,206,135) );
    //! [Draw the histograms for B, G and R]
    //! [Normalize the result to ( 0, histImage.rows )]
    cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    //! [Draw for each channel]
    for( int i = 1; i < histSize; i++ )
    {
        cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
              cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
              cv::Scalar( 0, 0, 139), 2, 8, 0  );
    }
    imshow("calcHist Camera Image Original", histImage );

    
    //switch(feature_type) {
    //    case 0 :
    //        std::cout << "FAST" << std::endl; 
            //------------FAST detect
            detector->detect(flipped_image,keypoints_orig,cv::noArray());
            std::cout<<"Number of Keypoints Detected Original= "<<keypoints_orig.size()<<std::endl;
            drawKeypoints(flipped_image,keypoints_orig,flipped_image_with_kp);
    //        break;
     //   case 1 :
     //       std::cout << "SURF" << std::endl;
    //        break;
     //   case 2 :
     //       std::cout << "SIFT" << std::endl;
    //        break;
    //    default :
    //        std::cout << "Invalid Feature Type" << std::endl;
   //}

    




    // Equilize the histogram
    cv::equalizeHist(flipped_image, hist_equalized_image); 
    // Plot the equalized image
    //?????

    obj_CLAHE->apply(flipped_image_smooth,CLAHE_image);                                                  // ------------Do piecewise HE
    CLAHE_image_hist_calc = CLAHE_image;
    //cv::GaussianBlur(CLAHE_image, CLAHE_image_smoothed, cv::Size(3,3), 1);   // Working 
    //cv::bilateralFilter(CLAHE_image, CLAHE_image_smoothed, 5,  2, 2);  //Not good
    //cv::medianBlur(CLAHE_image, CLAHE_image_smoothed, 3);

    detector->detect(CLAHE_image,keypoints_hist_equa,cv::noArray());
    std::cout<<"Number of Keypoints Detected after HE= "<<keypoints_hist_equa.size()<<std::endl;
    drawKeypoints(CLAHE_image,keypoints_hist_equa,hist_equalized_image_with_kp);
    Canny(flipped_image_smooth, edge_detected, lowThreshold, lowThreshold*ratio, kernel_size );
    //erode(flipped_image_smooth,edge_detected,3);

    res_cv.header = msg->header;
    res_cv.encoding = sensor_msgs::image_encodings::MONO8;
    res_cv.image =  CLAHE_image;




    //-----------Calculate and Plot CLAHE HIST
    split( CLAHE_image_hist_calc, g_plane_CLAHE );
    calcHist( &g_plane_CLAHE[0], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    cv::Mat histImage_CLAHE( hist_h, hist_w, CV_8UC3, cv::Scalar( 235,206,135) );
    //! [Draw the histograms for B, G and R]
    //! [Normalize the result to ( 0, histImage.rows )]
    cv::normalize(g_hist, g_hist, 0, histImage_CLAHE.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    //! [Draw for each channel]
    for( int i = 1; i < histSize; i++ )
    {
        cv::line( histImage_CLAHE, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
              cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
              cv::Scalar( 0, 0, 139), 2, 8, 0  );
    }
    imshow("calcHist Camera Image CLAHE", histImage_CLAHE );

   







    // Update GUI Window
    
    cv::imshow(OPENCV_WINDOW, flipped_image);
    cv::imshow("Image_CLAHE_Smoothened", flipped_image_smooth);
    
    cv::imshow(OPENCV_EDGE, edge_detected);

    /*
    cv::imshow(OPENCV_WINDOW_FAST,flipped_image_with_kp);
    cv::imshow(OPENCV_WINDOW_HIST_EQUALIZED, hist_equalized_image);
    cv::imshow(OPENCV_WINDOW_HIST_EQUALIZED_FAST, hist_equalized_image_with_kp);
    cv::imshow(OPENCV_WINDOW_CLAHE, CLAHE_image);
    */
    //cv::imshow("Image_CLAHE_Smoothened", CLAHE_image_smoothed);

    cv::waitKey(3);
    std::cout<<"End here \n";
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    image_pub_.publish(res_cv.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImagebridgeRosCV ic;
  ros::spin();
  return 0;
}
