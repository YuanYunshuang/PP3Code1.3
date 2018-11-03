#ifndef __FEATUREDETECTOR_H__
#define __FEATUREDETECTOR_H__

#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "./utils.h"

namespace pp3{

typedef std::pair<cv::Point2d, std::vector<double> > Discriptor;

	struct PointCloud{//registration point cloud
		XYZ_p cloud;
		pcl::PointXYZ center;
		double floor_height;
		cv::Mat density_map;	
		cv::Mat depth_map;
		cv::Mat planar_map;	//use pca to check if it a blob or linear region
		cv::Mat diff_normals_map;
		cv::Mat don_pca;
		cv::Mat don_nms;
		std::vector<cv::Point2d> PoIs;
		std::vector<Discriptor> discriptors;
		std::pair<int,int> t;
		double rot;
	};

	struct Map{//map point cloud
		XYZ_p cloud;
		cv::Mat density_map;	
		cv::Mat depth_map;
		cv::Mat planar_mapr;	//use pca to check if it a blob or linear region
		std::vector<cv::Point2d> PoIs;
		std::vector<Discriptor> discriptors;
	};

	void harrisCorners(PointCloud &pt, int thresh, cv::Mat &dst_norm){
		cv::Mat dst, dst_norm_scaled;
		dst = cv::Mat::zeros( pt.density_map.size(), CV_32FC1);
		// Detecting corners
		/*
		void cornerHarris(InputArray src, OutputArray dst, int blockSize, int ksize, double k, int borderType=BORDER_DEFAULT ) Parameters:
		src – Input single-channel 8-bit or floating-point image.
		dst – Image to store the Harris detector responses. It has the type CV_32FC1 and the same size as src .
		blockSize – Neighborhood size 
		ksize – Aperture parameter for the Sobel() operator.
		k – Harris detector free parameter. See the formula below.
		borderType – Pixel extrapolation method.
		*/

		cv::cornerHarris( pt.density_map, dst, 7, 5, 0.05, cv::BORDER_DEFAULT );

		// Normalizing
		cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
		cv::convertScaleAbs( dst_norm, dst_norm_scaled );

		// Drawing a circle around corners and store the points of interest
		cv::Point2d p;
		for( int j = 0; j < dst_norm.rows ; j++ ){
			 for( int i = 0; i < dst_norm.cols; i++ ){
				if( (int) dst_norm.at<float>(j,i) > thresh){
					//void circle(Mat& img, Point center, int radius, const Scalar& color, 
					//int thickness=1, int lineType=8, int shift=0)
			   		cv::circle( dst_norm_scaled, cv::Point(i, j),5, cv::Scalar(255), 1, 8, 0 );
					p.x = j; p.y = i;
					pt.PoIs.push_back(p);
				}
			 }
		}


		// Showing the result
		//cv::namedWindow( "corners_window", CV_WINDOW_AUTOSIZE );
		cv::imshow( "corners_norm", dst_norm_scaled );
		cv::waitKey(0);
		cv::destroyAllWindows();
		cv::imshow( "corners_norm_scaled", dst_norm_scaled );
		cv::waitKey(0);
		cv::destroyAllWindows();
    }

	void getDiscriptor(PointCloud &pt){
		

	}

	void getCylindricalDiscrip(PointCloud &pt, float rmin, float rmax, float delta_r, float delta_theta){

	}




}//end namespace pp3

#endif
