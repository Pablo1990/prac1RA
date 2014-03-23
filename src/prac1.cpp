#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings; 

cv_bridge::CvImagePtr img1;
cv_bridge::CvImagePtr img2;
cv::Mat result;
int frame = 0;
bool img1_exist = false;
bool img2_exist = false;
bool img_result = false;

class Prac1 {
	ros::Subscriber imgSub;

	public:Prac1(ros::NodeHandle& nh) { 

		imgSub = nh.subscribe("/camera/rgb/image_color", 1, &Prac1::imageCb, this);
	};


	void bucle() {
		ros::Rate rate(1); 
		while (ros::ok()) {
			ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
			rate.sleep(); // Espera a que finalice el ciclo
		}
	};

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		if(!img1_exist)
		{
			//Capturamos la primera imagen
			std::cerr<<" imagecb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
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
			img1 = cv_ptr;
			frame = msg->header.seq;
			img1_exist = true;
		}
		else if(!img2_exist)
		{
			//Capturamos la segunda imagen
			img2_exist = true;
			std::cerr<<" imagecb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
			cv_bridge::CvImagePtr cv_ptr;
			//Capturamos una copia de la segunda imagen
			//que despues utilizaremos como primera en el siguiente frame
			cv_bridge::CvImagePtr img_aux;
			try
			{
				//Copiamos la auxiliar y la segunda
				cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
				img_aux = cv_bridge::toCvCopy(msg, enc::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			//nos guardamos la segunda imagen
			img2 = cv_ptr;
			Mat src_gray1;
			Mat src_gray2;
			//Pasamos las dos imagenes de color a gray
			cvtColor( img1->image, src_gray1, CV_BGR2GRAY );
			cvtColor( img2->image, src_gray2, CV_BGR2GRAY );

			//The threshold
			//Cuanto mayor, mas esquina seran
			int minHessian = 400;

			//Declaramos el detector
			SurfFeatureDetector detector( minHessian );
	   	//SiftFeatureDetector detector;
	   	//FastFeatureDetector detector;
	   	//GoodFeaturesToTrackDetector detector;
	   	//MserFeatureDetector detector;
	   	//StarFeatureDetector detector;
	   	//DenseFeatureDetector detector;
	   	//SimpleBlobDetector detector;
			vector<KeyPoint> points1;
			vector<KeyPoint> points2;
			//el detector capturara los puntos clave
			//en algunos casos esquina otros simplemente puntos clave
			//de cada una imagenes
			detector.detect(src_gray1, points1);
			detector.detect(src_gray2, points2);

	    /*Extraer descriptores*/
			Mat descriptors_1, descriptors_2;
			//En comentarios estan todos los posibles descriptores
	    //SiftDescriptorExtractor extractor;
			//Declaramos el extractor de descriptores
			SurfDescriptorExtractor extractor;
	    //OrbDescriptorExtractor extractor;
	    //CalonderDescriptorExtractor extractor;
	    //OpponentColorDescriptorExtractor extractor;
	    //BriefDescriptorExtractor extractor;
			//a partir de los puntos, conseguimos los descriptores
			extractor.compute( src_gray1, points1, descriptors_1 );
			extractor.compute( src_gray2, points2, descriptors_2 );

  		/*El matching de puntos*/
  		/*FlannBasedMatcher matcher;*/
			vector<vector<DMatch> > matches;
	  	/* Distintos tipos de funciones de matchings: BruteForce (it uses L2 ), BruteForce-L1, BruteForce-Hamming, BruteForce-Hamming(2), FlannBased */
	  	//Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create( "FlannBased" );
	  	//FlannBasedMatcher descriptorMatcher;
	  	//Param: NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2
			//al parecer con NORM_L1 es con el que mejor funciona el surf
			BFMatcher descriptorMatcher(NORM_L1,false);
			//Capturamos las dos relaciones mas fuertes de 1 punto (sera con otro punto cada una)
			descriptorMatcher.knnMatch( descriptors_1, descriptors_2, matches, 2);


			/*------------------- Filtrado de matches ------------------------*/
			//El primer filtrado es que el primer mejor matching sea menor
			// que el segundo*0.8
			vector<DMatch> better_matches;
			for (int i=0; i<descriptors_1.rows; i++){
				if(matches[i][0].distance<(matches[i][1].distance*0.8)){
					better_matches.push_back(matches[i][0]);
				}
			}

			double max_dist = 0; double min_dist = 100;

		  //-- Quick calculation of max and min distances between keypoints
			for( int i = 0; i < better_matches.size(); i++ )
				{ double dist = better_matches[i].distance;
					if( dist < min_dist ) min_dist = dist;
					if( dist > max_dist ) max_dist = dist;
				}

				//printf("-- Max dist : %f \n", max_dist );
				//printf("-- Min dist : %f \n", min_dist );

		  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
		  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
		  //-- small)
		  //-- PS.- radiusMatch can also be used here.
				vector< DMatch > good_matches;
				for( int i = 0; i < better_matches.size(); i++ )
					{ if(better_matches[i].distance <= max(2*min_dist, 0.02) )
						{ good_matches.push_back( better_matches[i]); }
					}

					Mat img_matches;
					drawMatches( src_gray1, points1, src_gray2, points2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
						vector<char>(), DrawMatchesFlags::DRAW_RICH_KEYPOINTS  );

		  //-- Localize the object
					std::vector<Point2f> scena1;
					std::vector<Point2f> scene2;

					for( int i = 0; i < good_matches.size(); i++ )
					{
		    //-- Get the keypoints from the good matches
						scena1.push_back( points1[ good_matches[i].queryIdx ].pt );
						scene2.push_back( points2[ good_matches[i].trainIdx ].pt );
					}

				try{
					Mat H = findHomography( scena1, scene2, CV_RANSAC );
				
		  //-- Get the corners from the image_1 ( the object to be "detected" )
					std::vector<Point2f> obj_corners(4);
					obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( src_gray1.cols, 0 );
					obj_corners[2] = cvPoint( src_gray1.cols, src_gray1.rows ); obj_corners[3] = cvPoint( 0, src_gray1.rows );
					std::vector<Point2f> scene_corners(4);

					perspectiveTransform( obj_corners, scene_corners, H);

		  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
					line( img_matches, scene_corners[0] + Point2f( src_gray1.cols, 0), scene_corners[1] + Point2f( src_gray1.cols, 0), Scalar(0, 255, 0), 4 );
					line( img_matches, scene_corners[1] + Point2f( src_gray1.cols, 0), scene_corners[2] + Point2f( src_gray1.cols, 0), Scalar( 0, 255, 0), 4 );
					line( img_matches, scene_corners[2] + Point2f( src_gray1.cols, 0), scene_corners[3] + Point2f( src_gray1.cols, 0), Scalar( 0, 255, 0), 4 );
					line( img_matches, scene_corners[3] + Point2f( src_gray1.cols, 0), scene_corners[0] + Point2f( src_gray1.cols, 0), Scalar( 0, 255, 0), 4 );

		  //-- Show detected matches
					imshow( "Good Matches & Object detection", img_matches );
					const std::vector<Point2f> points_ant_transformed(points1.size());
					std::vector<Point2f> keypoints_ant_vector(points1.size());
					cv::KeyPoint::convert(points1,keypoints_ant_vector);

		//transformamos los puntos de la imagen anterior
					perspectiveTransform( keypoints_ant_vector, points_ant_transformed, H);

		//creamos una copia de la imagen actual que usaremos para dibujar
					Mat transformed_image;
					cvtColor(src_gray1, transformed_image, CV_GRAY2BGR);

		//los que esten mas lejos que este parametro se consideran outliers (o que la transformacion está mal calculada)
		//este valor es orientativo, podeis cambiarlo y ajustarlo a los valores
					float distance_threshold=10.0; 
					int contdrawbuenos=0;
					int contdrawmalos=0;
					for ( int i =0;i<good_matches.size();i++)
					{
						int ind        = good_matches.at(i).trainIdx ;
						int ind_Ant    = good_matches.at(i).queryIdx;

						cv::Point2f p=        points2.at(ind).pt;
						cv::Point2f p_ant=    points_ant_transformed[ind_Ant];

					    circle( transformed_image, p_ant, 5, Scalar(255,0,0), 2, 8, 0 ); //ant blue
					    circle( transformed_image, p, 5, Scalar(0,255,255), 2, 8, 0 ); //current yellow

					    Point pointdiff = p - points_ant_transformed[ind_Ant];
					    float distance_of_points=cv::sqrt(pointdiff.x*pointdiff.x + pointdiff.y*pointdiff.y);

					    if(distance_of_points < distance_threshold){ // los good matches se pintan con un circulo verde mas grand
					    	contdrawbuenos++;
					        circle( transformed_image, p, 9, Scalar(0,255,0), 2, 8, 0 ); //current red
					    }
					    else{
					    	contdrawmalos++;
					    	line(transformed_image,p,p_ant,Scalar(0, 0, 255),1,CV_AA);
					    }
					}

					imshow( "transformed", transformed_image );
					imwrite("~/ejemplowrite.png",transformed_image );

					//cv::Mat result;
					warpPerspective(src_gray1,result,H,cv::Size(src_gray1.cols+src_gray2.cols,src_gray1.rows));
					cv::Mat half(result,cv::Rect(0,0,src_gray2.cols,src_gray2.rows));
					src_gray2.copyTo(half);
					imshow( "Easy Merge Result", result );
					img2_exist = false;
					img1 = img_aux;

					cv::waitKey(3);
				}
				catch(cv::Exception &e)
				{
					ROS_ERROR("Excepction: %s\nSkipping frame...", e.what());
					img2_exist = false;
					cv::waitKey(3);
				}
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

