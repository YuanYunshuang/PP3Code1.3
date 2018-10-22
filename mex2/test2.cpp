#include "mex.h"
#include "matrix.h"
#include "stdio.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
void mexFunction(int nlhs,mxArray *plhs[],int nrhs, const mxArray *prhs[]){

	 if (nrhs != 2) {
    mexErrMsgTxt("Two inputs required.");
	 return;
	 }
mxArray *xdata ,*out;
int i,j,r,c;
double avg;

xdata = mxDuplicateArray(prhs[0]);
double *a ,*b;
a=mxGetPr(xdata);
r = mxGetN(xdata);
c = mxGetM(xdata);
char * s =mxArrayToString(prhs[1]);
for(i=0;i<r;i++)
{
    avg=0.0;
    for(j=0;j<c;j++){
        avg +=a[i*c+j]; 
        //b[i*c+j]= a[i*c+j];
    }
    avg = avg/(double)c;
    printf("avg of column %d is %f a=%f\n",i+1, avg,a[i*c+j-1]);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PLYReader Reader;

 printf("read cloud %s \n",s);
 pcl::io::loadPLYFile(s,*cloud);
//Reader.read("test.ply", *cloud);
     printf("read cloud!\n");
    printf("reed cloud %d\n",cloud->points.size());
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {

	}
/*plhs[0]=mxCreateDoubleMatrix(1,cloud->points.size()*3,mxREAL);
out=plhs[0];
b=mxGetPr(out);
for(int i=0;i<cloud->points.size()*3;i+=3){
	    printf("avg of column %d is %f %f %f\n",i,cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
    b[i]=cloud->points[i].x;
	b[i+1]=cloud->points[i].y;
	b[i+2]=cloud->points[i].z;
}*/

}