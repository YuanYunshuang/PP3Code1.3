
#include "./featureDetector.h"
#include "./getMaps.h"
#include <stdlib.h>

//using namespace std;
using namespace pp3;


int main(int argc, char ** argv) {
	if (argc <4) {
		std::cout << "Need one path to pointcloud, one path for saving images and image size." << std::endl;
	}
	string pt_path = argv[1];
	string im_path = argv[2];
	int im_size = atoi(argv[3]);
	//get the file paths of the point clouds
	vector<string> files = getFiles(pt_path);

	//implemeting for each file: translate point cloud into 2D image
	PointCloud pt;
	pt.cloud =boost::make_shared <XYZ> ();
	pcl::PLYReader Reader;
	string path;
	for(int i=0; i<files.size(); i++){
		cout<<pt_path +"/"+ files[i]<<endl;
		Reader.read(pt_path +"/"+ files[i], *pt.cloud);
		cout << pt.cloud->points.size() << endl;
		if(pt.cloud->points.size() > 500000)
			filterPointCloud(pt.cloud);
		calCenter(pt.cloud, pt.center);
		pt.floor_height = calFloorHeight(pt.cloud, pt.center, 0.3);
		densityMap(pt,im_size);
		depthMap(pt,im_size);
		planarMap(pt, 0.5, 0.03,im_size);
		
		path = files[i];
		path = path.substr(0, path.length() - 4);
		string path_density_map = im_path + "/images/density/" + path + ".jpg";
		string path_depth_map = im_path + "/images/depth/" + path + ".jpg";
		string path_planar_map = im_path + "/images/curvature/" + path + ".jpg";
		cout<<"Saving images:"<<endl;
		cout<<path_density_map<<endl;
		cv::imwrite(path_density_map,pt.density_map);
		cout<<path_depth_map<<endl;
		cv::imwrite(path_depth_map,pt.depth_map);
		cout<<path_planar_map<<endl;
		cv::imwrite(path_planar_map,pt.planar_map);
		
		//cv::Mat dst_norm;
		//harrisCorners(pt, 150, dst_norm);
		
	}

	return 1;
}
