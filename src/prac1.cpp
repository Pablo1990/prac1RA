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

	   	/*Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");*/
	   	SiftFeatureDetector detector;
	    vector<KeyPoint> points1;
	    vector<KeyPoint> points2;
	    detector.detect(src_gray1, points1);
	    detector.detect(src_gray2, points2);

	    /*Extraer descriptores*/
	    Mat descriptors_1, descriptors_2;
	    //Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SIFT");
	    SiftDescriptorExtractor extractor;
      	extractor.compute( src_gray1, points1, descriptors_1 );
  		extractor.compute( src_gray2, points2, descriptors_2 );

  		/*Matcher*/
  		/*FlannBasedMatcher matcher;*/
	  	vector<vector<DMatch> > matches;
	  	/* Tipos: BruteForce (it uses L2 ), BruteForce-L1, BruteForce-Hamming, BruteForce-Hamming(2), FlannBased */
	  	//Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create( "FlannBased" );
	  	FlannBasedMatcher descriptorMatcher;

  		descriptorMatcher.knnMatch( descriptors_1, descriptors_2, matches, 2);

  		vector<DMatch> better_matches;
  		for (int i=0; i<descriptors_1.rows; i++){
  			//std::cout<<matches[i].size;
  			if(matches[i][0].distance<(matches[i][1].distance*0.8))
  				better_matches.push_back(matches[i][0]);
  		}

  		/*double max_dist = 0; double min_dist = 100;

		  //-- Quick calculation of max and min distances between keypoints
		  for( int i = 0; i < descriptors_1.rows; i++ )
		  { double dist = better_matches[i].distance;
		    if( dist < min_dist ) min_dist = dist;
		    if( dist > max_dist ) max_dist = dist;
		  }

		  printf("-- Max dist : %f \n", max_dist );
		  printf("-- Min dist : %f \n", min_dist );

		  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
		  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
		  //-- small)
		  //-- PS.- radiusMatch can also be used here.
		  std::vector< DMatch > good_matches;

		  for( int i = 0; i < descriptors_1.rows; i++ )
		  { if(better_matches[i].distance <= max(2*min_dist, 0.02) )
		    { good_matches.push_back( better_matches[i]); }
		  }
		*/
	  	Mat img_matches;
  		drawMatches( src_gray1, points1, src_gray2, points2,
               better_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
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

