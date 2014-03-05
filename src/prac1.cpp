#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings; 

cv_bridge::CvImagePtr img1;
cv_bridge::CvImagePtr img2;
bool img1_exist = false;
bool img2_exist = false;

class Prac1 {
	ros::Subscriber imgSub;
	
	public:Prac1(ros::NodeHandle& nh) { 
		
		imgSub = nh.subscribe("/camera/rgb/image_color", 1, &Prac1::imageCbFast, this);
	};


	void bucle() {
		ros::Rate rate(1); 
		while (ros::ok()) {
			cout<<imgSub<<endl;
			ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
			rate.sleep(); // Espera a que finalice el ciclo
		}
	};

	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
	   //primero las convertimos a un tipo de imagen de openCV
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
	    }

		 Mat src_gray;
		  Mat dst, dst_norm, dst_norm_scaled;
		  dst = Mat::zeros( src_gray.size(), CV_32FC1 );

		//obtener la imagen en gris
		 cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

		 /// Detector parameters
		  int blockSize = 2;
		  int apertureSize = 3;
		  double k = 0.04;


		  /// Detecting Harris corners
		  cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

		  /// Normalizing
		  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
		  convertScaleAbs( dst_norm, dst_norm_scaled );

		//miramos los valores calculados por Harris y si es superiror a un umbral ( thresh) dibujamos un circulo centrado en el 
		 int thresh = 200;
		  /// Drawing a circle around corners
		  for( int j = 0; j < dst_norm.rows ; j++ )
		     { for( int i = 0; i < dst_norm.cols; i++ )
			  {
			    if( (int) dst_norm.at<float>(j,i) > thresh ) // si el valor (i,j) de la imagen es mayor que el umbral
			      {
			       circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
			      }
			  }
		     }
		  /// Showing the result
		 //esta instruccion es para ajustar la ventana a pintar
		  namedWindow( "Corners detected", CV_WINDOW_AUTOSIZE );

		 //dibujamos la imagen dst_norm_scaled en la ventana
		  imshow( "Corners detected", dst_norm_scaled );

		cv::waitKey(3);	

	}

//Callback Ejemplo usando el detector FAST:


void imageCbFast(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    std::cerr<<" imagecb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;

    if(!img1_exist)
    {
    	img1 = cv_ptr;
    	img1_exist = true;
    }
    else if(!img2_exist)
    {
    	img2 = cv_ptr;
    	img2_exist = true;
    }
    else
    {
	  	Mat src_gray1;
	  	Mat src_gray2;

	 	cvtColor( img1->image, src_gray1, CV_BGR2GRAY );
	 	cvtColor( img2->image, src_gray2, CV_BGR2GRAY );

	   	Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");
	    vector<KeyPoint> points1;
	    vector<KeyPoint> points2;
	    detector->detect(src_gray1, points1);
	    detector->detect(src_gray2, points2);

	    /*Extraer descriptores*/
	    Mat descriptors_1, descriptors_2;
	    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SIFT");
      	extractor->compute( src_gray1, points1, descriptors_1 );
  		extractor->compute( src_gray2, points2, descriptors_2 );

  		/*Matcher*/
  		FlannBasedMatcher matcher;
	  	std::vector< DMatch > matches;
	  	matcher.match( descriptors_1, descriptors_2, matches );

	  	Mat img_matches;
  		drawMatches( src_gray1, points1, src_gray2, points2,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	    /*Mat imageColor1;
	    Mat imageColor2;
	    cvtColor(src_gray1, imageColor1, CV_GRAY2BGR);
	    cvtColor(src_gray2, imageColor2, CV_GRAY2BGR);
	    
	    for (size_t i = 0; i < points1.size(); i++)
	    {
	      circle(imageColor1, points1[i].pt, 3, CV_RGB(255, 0, 0));
	    }

	    for (size_t i = 0; i < points2.size(); i++)
	    {
	      circle(imageColor2, points2[i].pt, 3, CV_RGB(255, 0, 0));
	    }*/
	    //se pueden pintar tambien con esta funcion
	    //drawKeypoints(imageColor, points, imageColor, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_OVER_OUTIMG);

	    imshow("SIFT keypoints", img_matches);
	    
	    cv::waitKey(3);   	
    }
  }
};



int main(int argc, char **argv) {
	ros::init(argc, argv, "prac1"); // Inicializa un nuevo nodo llamado wander
	ros::NodeHandle nh;
	Prac1 prac1(nh); // Crea un objeto de esta clase y lo asocia con roscore
	prac1.bucle(); // Ejecuta el bucle
	return 0;
};

