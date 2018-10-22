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

typedef pcl::PointCloud<pcl::PointXYZ> XYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_p;

vector<string> getFiles(string cate_dir)
{
	int len;
	vector<string> files;//container for file names 
	struct dirent *pDirent;
	DIR *pDir;

	pDir = opendir(cate_dir.c_str());
	if (pDir == NULL) {
		printf("Cannot open directory '%s'\n", cate_dir.c_str());
	}

	while ((pDirent = readdir(pDir)) != NULL) {
		//cout<< string(pDirent->d_name)<<endl;
		if ((pDirent->d_name[0]) != '.')
			files.push_back(string(pDirent->d_name));
	}
	closedir(pDir);
	//sort(files.begin(), files.end());
	return files;
}

cv::Mat getGridMap(XYZ_p cloud, double resolution) {
	
	cv::Mat img(1000, 1000, CV_8UC1, cv::Scalar(0));
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

	return img;
}

XYZ_p filterPointCloud(XYZ_p){
	XYZ_p cloud_filtered(new XYZ);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.1f, 0.1f, 0.1f);
	sor.filter(*cloud_filtered);
	
	return cloud_filtered;
}

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

	XYZ_p cloud1(new XYZ);
	XYZ_p cloud2(new XYZ);
	pcl::PLYReader Reader;
	Reader.read(argv[1], *cloud1);
	Reader.read(argv[2], *cloud2);
	cout << cloud1->points.size() << endl;
	cout << cloud2->points.size() << endl;
	cloud_filtered = filterPointCloud(cloud2);
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
