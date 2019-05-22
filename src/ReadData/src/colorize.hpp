#ifndef __COLORIZE_HPP__
#define __COLORIZE_HPP__

#include <pcl/features/normal_3d_omp.h>


//-------------------------------------------------------
template<typename PointType>
double computeCloudResolution (const typename pcl::PointCloud<PointType>::ConstPtr &cloud)
//-------------------------------------------------------
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud (cloud);

#pragma omp parallel for
	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);

#pragma omp critical
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

//-------------------------------------------------------
template<typename PointT>
void computeNormals(
		typename pcl::PointCloud<PointT>::ConstPtr cloud,
		typename pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
		float radius = 0.03) // radius 3cm
//-------------------------------------------------------
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Use all neighbors in a sphere of radius
	ne.setRadiusSearch (radius);

	// Compute the features
	ne.compute (*cloud_normals);
}

//-------------------------------------------------------
template<typename PointT>
void addColors(
		typename pcl::PointCloud<PointT>::ConstPtr cloud_in,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
//-------------------------------------------------------
{
	std::cout << "Compute resolution..." << std::endl;
	double resolution = computeCloudResolution<PointT>(cloud_in);

	// Estimate normals
	std::cout << "Compute normals..." << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	computeNormals<PointT>(cloud_in, normals, resolution*3);

	cloud_out->resize(cloud_in->size());

	Eigen::Vector3f lightSource(0,0,0);

	// Set color based on light source
	std::cout << "Compute colors..." << std::endl;
#pragma omp parallel for
	for(unsigned int i = 0; i < cloud_in->size(); i++)
	{
		Eigen::Vector3f v((*cloud_in)[i].x, (*cloud_in)[i].y, (*cloud_in)[i].z);
		Eigen::Vector3f n((*normals)[i].normal_x, (*normals)[i].normal_y, (*normals)[i].normal_z);

		Eigen::Vector3f vec = (lightSource - v).normalized();

		float weight = std::abs(vec.dot(n));

		int br = (int)(205 * weight) + 50;
		br = std::max(0, std::min (255, br));

		(*cloud_out)[i].x = v(0);
		(*cloud_out)[i].y = v(1);
		(*cloud_out)[i].z = v(2);
		(*cloud_out)[i].r = br;
		(*cloud_out)[i].g = br;
		(*cloud_out)[i].b = br;
	}
}




#endif // __COLORIZE_HPP__
