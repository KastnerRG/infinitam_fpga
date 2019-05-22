/*
 * depth_map.hpp
 *
 *  Created on: May 14, 2016
 *      Author: qkgautier
 */

#ifndef SRC_DEPTH_MAP_HPP_
#define SRC_DEPTH_MAP_HPP_


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "CImg.h"

#include "utils.h"


namespace tangodata
{

//-------------------------------------------------------
template<typename T>
T float_to_int(float v)
{
	return static_cast<T>(roundf(v));
}
template<> float float_to_int(float v){ return v;}
template<> double float_to_int(float v){ return v;}

template<typename T>
T float_to_int(double v)
{
	return static_cast<T>(round(v));
}
template<> float float_to_int(double v){ return v;}
template<> double float_to_int(double v){ return v;}

//-------------------------------------------------------


//-------------------------------------------------------
template<typename DepthType>
void depthMapToPointCloud(
		const DepthType* depth_map,
		const Intrinsics& depth_intr,
		pcl::PointCloud<pcl::PointXYZ>& points,
		float min_depth = 300.f,    // set to 0 if not normalized
		float max_depth = 3000.f,   // set to 0 if not normalized
		float depth_factor = 1000.f, // convert from (raw data or min/max_depth unit) to (meters)
		bool reverse = false)
//-------------------------------------------------------
{
	points.clear();
	points.reserve(depth_intr.height * depth_intr.width);

	bool is_normalized = (max_depth > min_depth);

	for(uint32_t v = 0; v < depth_intr.height; v++)
	{
		for(uint32_t u = 0; u < depth_intr.width; u++)
		{
			DepthType depth = depth_map[v * depth_intr.width + u];

			if (depth != 0)
			{
				if(is_normalized)
				{
					depth = (float)depth / std::numeric_limits<DepthType>::max();
					if(reverse){ depth = 1.f - depth; }
					depth = depth * (max_depth - min_depth) + min_depth;
				}
				else if(reverse)
				{
					depth = std::numeric_limits<DepthType>::max() - depth;
				}

				float z = depth / depth_factor;

				float vx = z * (u - depth_intr.cx) * (1.f / depth_intr.fx);
				float vy = z * (v - depth_intr.cy) * (1.f / depth_intr.fy);
				float vz = z;

				points.push_back(pcl::PointXYZ(vx, vy, vz));

			}
		}
	}
}


//-------------------------------------------------------
struct DepthMapCreator
//-------------------------------------------------------
{
	uint32_t kernel_size;
	float min_depth;
	float max_depth;
	float depth_factor;
	bool reverse;
	unsigned int bg_sparse_factor;
	unsigned int bg_border_size;

	DepthMapCreator():
		kernel_size(1),
		min_depth(300.f),
		max_depth(3000.f),
		depth_factor(1000.f),
		reverse(false),
		bg_sparse_factor(8),
		bg_border_size(2)
	{}

	template<typename DepthType, typename MaskType>
	void pointsToDepthMap(
			const pcl::PointCloud<pcl::PointXYZ>& points,
			const Intrinsics& depth_intr,
			std::vector<DepthType>& depth_map,
			std::vector<MaskType>& mask,
			DepthInterpolationType interpolate = DEPTH_INTERP_NONE,
			float z_offset = 0)
	{
		depth_map.clear();
		mask.clear();

		depth_map.resize(depth_intr.width * depth_intr.height, std::numeric_limits<DepthType>::quiet_NaN());
		mask.resize(depth_intr.width * depth_intr.height, 0);

		int box_x1 = 0, box_x2 = 0;
		int box_y1 = 0, box_y2 = 0;

		if(interpolate == DEPTH_INTERP_NN)
		{
			box_x1 = -kernel_size; box_x2 = kernel_size;
			box_y1 = -kernel_size; box_y2 = kernel_size;
		}


		bool truncateDepth = (max_depth > min_depth);

		for(uint32_t i = 0; i < points.size(); i++)
		{
			const float x = points[i].x;
			const float y = points[i].y;
			const float z = points[i].z + z_offset;

//			std::cout << x << " " << y << " " << z << std::endl;

			unsigned int u = (unsigned int)roundf((x * depth_intr.fx) / z + depth_intr.cx);
			unsigned int v = (unsigned int)roundf((y * depth_intr.fy) / z + depth_intr.cy);
			float d = z * depth_factor;

//			std::cout << u << " " << v << " " << d << std::endl;
//			std::cout << std::endl;

			DepthType max_value = std::numeric_limits<DepthType>::max();

			if(u < depth_intr.width && v < depth_intr.height)
			{
				for(int a = box_y1; a <= box_y2; a++)
				{
					for(int b = box_x1; b <= box_x2; b++)
					{
						int index = (v+a) * depth_intr.width + u+b;
						if(index >= 0 && index < depth_intr.width * depth_intr.height)
						{
							DepthType value = depth_map[index];

							//if(value){ d = (value + d) / 2; }
							if(std::numeric_limits<DepthType>::has_quiet_NaN)
							{
								if(!std::isnan(value)){ continue; }
							}
							else
							{
								if(value != std::numeric_limits<DepthType>::quiet_NaN()){ continue; }
							}


							DepthType newvalue;

							if(truncateDepth)
							{
								newvalue = float_to_int<DepthType>(
										std::min((double)max_value,
												std::max(0.0,
														( double(d-min_depth) / (max_depth-min_depth) ) * max_value)));
							}
							else{ newvalue = float_to_int<DepthType>(d); }

							if(reverse){ newvalue = max_value - newvalue; }

							depth_map[index] = newvalue;
							mask[index] = std::numeric_limits<MaskType>::max();
						}
					}
				}
			}
		}// for each point

		if(interpolate == DEPTH_INTERP_BILINEAR)
		{
			std::vector<DepthType> depth_map_interp;
			interpolateBilinear(depth_intr, depth_map, depth_map_interp, mask);
			depth_map_interp.swap(depth_map);
		}
	}

	template<typename DepthType, typename MaskType>
	void interpolateBilinear(
			const Intrinsics& depth_intr,
			const std::vector<DepthType>& depth_map,
			std::vector<DepthType>& depth_map_interp,
			std::vector<MaskType>& mask)
	{
		depth_map_interp.clear();
		mask.clear();
		depth_map_interp.resize(depth_intr.width * depth_intr.height);
		mask.resize(depth_intr.width * depth_intr.height, 0);

		const double sigma_color = 30;     //in mm
		//const double sigma_space = 4.5;     // in pixels
		const double sigma_space = kernel_size;     // in pixels

		const double sigma_space2_inv_half = 0.5f / (sigma_space * sigma_space);
		const double sigma_color2_inv_half = 0.5f / (sigma_color * sigma_color);

		const int R = static_cast<int>(sigma_space * 1.5);
		const int D = R * 2 + 1;
		
        for(int y = 0; y < depth_intr.height; y++)
		{
			for(int x = 0; x < depth_intr.width; x++)
			{
				double value = depth_map[y * depth_intr.width + x];
                if(std::isnan(value)){ value = 0; }

				int tx = std::min (x - D / 2 + D, (int)depth_intr.width - 1);
				int ty = std::min (y - D / 2 + D, (int)depth_intr.height - 1);

				double sum1 = 0;
				double sum2 = 0;

				for (int cy = std::max (y - D / 2, 0); cy < ty; ++cy)
				{
					for (int cx = std::max (x - D / 2, 0); cx < tx; ++cx)
					{
						double tmp = depth_map[cy * depth_intr.width + cx];
						if(tmp == 0 || std::isnan(tmp)){ continue; }

						double space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
						double color2 = (value - tmp) * (value - tmp);
						if(value == 0){ color2 = 0; }

						double weight = exp (-(space2 * sigma_space2_inv_half + color2 * sigma_color2_inv_half));

						sum1 += tmp * weight;
						sum2 += weight;
					}
				}

				double res = sum1 / sum2;

				res = std::max (0.0, std::min (res, (double)std::numeric_limits<DepthType>::max ()));

				depth_map_interp[y * depth_intr.width + x] = float_to_int<DepthType>(res);
				mask            [y * depth_intr.width + x] = std::numeric_limits<MaskType>::max();
			}
		}
	}


	template<typename DepthType>
	void setBackground(
			const unsigned char* background_mask,
			const Intrinsics& depth_intr,
			DepthType* depth_map,
			DepthType bg_value = 1,
			int bg_width = 0,
			int bg_height = 0)
	{
		bg_sparse_factor = std::max(1u, bg_sparse_factor);

		if(bg_width <= 0){ bg_width = depth_intr.width; }
		if(bg_height <= 0){ bg_height = depth_intr.height; }

		for(int y = 0; y < depth_intr.height; y+=bg_sparse_factor)
		{
			int y_mask = roundf(y * ((float)bg_height / depth_intr.height));
			if(y_mask < bg_border_size || y_mask >= bg_height-bg_border_size){ continue; }

			for(int x = 0; x < depth_intr.width; x+=bg_sparse_factor)
			{
				int x_mask = roundf(x * ((float)bg_width / depth_intr.width));
				if(x_mask < bg_border_size || x_mask >= bg_width-bg_border_size){ continue; }

				if(background_mask[y_mask * bg_width + x_mask])
				{
					depth_map[y * depth_intr.width + x] = bg_value;
				}
			}
		}
	}

	template<typename T>
	void processBackgroundMask(
			const T* background_mask,
			int bg_width,
			int bg_height,
			T* output)
	{
		for(unsigned int y = 0; y < bg_height; y++)
		{
			for(unsigned int x = 0; x < bg_width; x++)
			{
				int index = y * bg_width + x;

				T value = background_mask[index];
				if(value){ value = 0; }
				else{ value = 1; }

				int blob_size = 0;

				for(unsigned int cy = std::max(bg_border_size, y-1); cy < std::min(bg_height-bg_border_size, y+2); cy++)
				{
					for(unsigned int cx = std::max(bg_border_size, x-1); cx < std::min(bg_width-bg_border_size, x+2); cx++)
					{
						int cindex = cy * bg_width + cx;
						if(background_mask[cindex] == 0){ blob_size++; }
					}
				}
				if(blob_size <= 1){ value = 0; }
				output[index] = value;
			}
		}
	}
};



//-------------------------------------------------------
template<typename DepthType, typename MaskType>
void depthMapToTransparentImg(
		std::vector<DepthType>& depth_map,
		std::vector<MaskType>& mask,
		uint32_t width,
		uint32_t height,
		cimg_library::CImg<DepthType>& img)
//-------------------------------------------------------
{
	img.assign(width, height, 1, 4);

	for(uint32_t y = 0; y < height; y++)
	{
		for(uint32_t x = 0; x < width; x++)
		{
			uint32_t index = y * width + x;

			DepthType value = depth_map[index];

			img(x,y,0,0) = value;
			img(x,y,0,1) = value;
			img(x,y,0,2) = value;
			img(x,y,0,3) = mask[index];
		}
	}
}

} // namespace tangodata

#endif /* SRC_DEPTH_MAP_HPP_ */
