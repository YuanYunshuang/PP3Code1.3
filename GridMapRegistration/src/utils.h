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


namespace pp3{
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



XYZ_p filterPointCloud(XYZ_p cloud){
	XYZ_p cloud_filtered(new XYZ);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.1f, 0.1f, 0.1f);
	sor.filter(*cloud_filtered);
	
	return cloud_filtered;
}

}//end namespace pp1.3
