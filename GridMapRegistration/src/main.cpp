
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

		pt.floor_height = calFloorHeight(pt.cloud, pt.center, 0.2);
		densityMap(pt,im_size);
		depthMap(pt,im_size);
		//planarMap(pt, 30, 0.03,im_size);
		//deffNormalsMap(pt, 0.2,0.4, 0.3, im_size);
		
		path = files[i];
		path = path.substr(0, path.length() - 4);

		string path_density_map = im_path + "/images/density_pca/" + path + ".jpg";
		string path_depth_map = im_path + "/images/depth_pca/" + path + ".jpg";
		//string path_planar_map = im_path + "/images/curvature/" + path + ".jpg";
		string path_don_map = im_path + "/images/diffNorm/" + path + ".jpg";
		string path_don_pca = im_path + "/images/diffNormPCA/" + path + ".jpg";
		string path_don_nms = im_path + "/images/diffNormNM/" + path + ".jpg";
		cout<<"Saving images:"<<endl;
		cv::Mat img(im_size, im_size,CV_8UC1, cv::Scalar(0));
		//cout<<path_density_map<<endl;
		PCA2d(pt.density_map, img, 30, 0.3);
		pt.density_map = img;
		cv::imshow("density_map1",pt.density_map);
		cv::waitKey(0);
		PCA2d(pt.density_map, img, 20, 0.3);
		pt.density_map = img;
		cv::imshow("density_map2",pt.density_map);
		cv::waitKey(0);
		//PCA2d(pt.density_map, pt.density_map, 5, 0.1);
		cv::imwrite(path_density_map,pt.density_map);
		//cout<<path_depth_map<<endl;
		PCA2d(pt.depth_map, img, 40, 0.3);
		pt.depth_map = img;
		cv::imshow("depth_map1",img);
		cv::waitKey(0);
		PCA2d(pt.depth_map, img, 20, 0.3);
		pt.depth_map = img;
		cv::imshow("depth_map2",img);
		cv::waitKey(0);
		//PCA2d(pt.depth_map, pt.depth_map, 5, 0.1);
		cv::imwrite(path_depth_map,pt.depth_map);

		//cout<<path_don_map<<endl;
		//cv::imwrite(path_don_map,pt.diff_normals_map);
		//PCA2d(pt.diff_normals_map, pt.don_pca, 200, 0.3);
		//PCA2d(pt.don_pca, pt.don_pca, 100, 0.3);
		//cv::imshow("After PCA",pt.don_pca);
		//cv::waitKey(0);
		//cv::imwrite(path_don_pca,pt.don_pca);
		//NonMaxSurpression(pt.diff_normals_map, pt.don_nms, 15, 15);
		//cv::imshow("NMS",pt.don_nms);
		//cv::waitKey(0);
		//cv::imwrite(path_don_nms,pt.don_nms);
		cv::destroyAllWindows();

		//cv::Mat dst_norm;
		//harrisCorners(pt, 150, dst_norm);
		
	}

	return 1;
}
