
#include "colorize.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace pcl;
using namespace std;

//-------------------------------------------------------
int main (int argc, char** argv)
//-------------------------------------------------------
{
	const char* input_filename  = argv[1];
	const char* output_filename = argv[2];
	
	PointCloud<PointXYZ>::Ptr points_ptr(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>& points = *points_ptr;

	pcl::io::loadPCDFile<PointXYZ>(input_filename, points);

	if(points.empty()){ pcl:io::loadPLYFile<PointXYZ>(input_filename, points); }

	PointCloud<PointXYZRGB>::Ptr out_points_ptr(new PointCloud<PointXYZRGB>);

	addColors<PointXYZ>(points_ptr, out_points_ptr);

	pcl::io::savePCDFile(output_filename, *out_points_ptr);

	return 0;
}
