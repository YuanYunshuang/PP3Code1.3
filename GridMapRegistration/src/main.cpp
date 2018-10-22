#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <limits>
using namespace std;

void showGridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution,
		string name, cv::Mat img) {

	double min_z = numeric_limits<double>::max();
	pcl::PointXYZ center(0,0,0);
	for (int i = 0; i < cloud->points.size(); i++) {
		center.x += cloud->points[i].x;
		center.y += cloud->points[i].y;
		center.z += cloud->points[i].z;
//		if (cloud->points[i].z < min_z) {
//			min_z = cloud->points[i].z;
//		}
	}
	center.x /= cloud->points.size();
	center.y /= cloud->points.size();
	center.z/=cloud->points.size();
	cout << min_z << endl;
	for (int i = 0; i < cloud->points.size(); i++) {
		pcl::PointXYZ p = cloud->points[i];
		cv::Point2d ip((p.x - center.x) / resolution + img.cols / 2,
				img.rows / 2 - (p.y - center.y) / resolution);
		if (p.z > center.z&&p.z<center.z+1 && ip.x < img.cols && ip.y < img.rows && ip.x > 0
				&& ip.y > 0) {
			img.at<char>(ip.x, ip.y) = 255;
		}
	}
	cv::imshow(name, img);
}

int main(int argc, char ** argv) {
	if (argc < 3) {
		std::cout << "Need two path to two pointclouds" << std::endl;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(argv[1], *cloud1);
	Reader.read(argv[2], *cloud2);
	cout << cloud1->points.size() << endl;
	cout << cloud2->points.size() << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud2);
	sor.setLeafSize(0.1f, 0.1f, 0.1f);
	sor.filter(*cloud_filtered);
	double resolution = 0.1; //one pixel =0.1m
	cv::Mat img1(1000, 1000, CV_8UC1, cv::Scalar(0));
	cv::Mat img2(1000, 1000, CV_8UC1, cv::Scalar(0));
	showGridMap(cloud1, resolution, "cloud1", img1);
	showGridMap(cloud_filtered, resolution, "cloud2", img2);
	cv::waitKey(0);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud1);

	while (!viewer.wasStopped()) {
	}
	return 1;
}
