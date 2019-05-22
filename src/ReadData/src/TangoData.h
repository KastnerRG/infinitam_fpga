/*
 * TangoData.h
 *
 *  Created on: Apr 17, 2016
 *      Author: qkgautier
 */

#ifndef TANGODATA_H_
#define TANGODATA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <map>


namespace tangodata
{

struct SyncedData
{
	double timestamp;
	Eigen::Affine3f pose;
	std::string depth;
	std::string color;
};


class TangoData
{
public:

	TangoData();
	virtual ~TangoData();

	void clear();

	bool load(const std::string& dir_path, std::string input_depth_dir = "");

	/**
	 * Remove some data so that all timestamps start roughly at the same time.
	 */
	void syncFirstTimestamp();

	/**
	 * Transform poses such that the first pose is an identity.
	 */
	void transformPoses();

	bool hasColors(){ return !color_paths_.empty(); }

	/**
	 * Get synced data based on depth. (i.e. try to sync color and pose data to each depth data)
	 * If the time difference between synced data and depth data is greater than time_threshold, skip.
	 */
	void getSyncedData(std::vector<SyncedData>& alldata, double time_threshold = 0.2);

	bool isObj() const { return is_obj_; }
	bool isImg() const { return is_img_; }


protected:

	/**
	 * Get data closest to the provided time (nearest neighbor).
	 * If the data time is too far from the provided time (abs(data time - provided time) > threshold), return null.
	 */
	template<class T>
	const T* getDataAtTime(
			const std::map<double, T>& data_map,
			double time,
			double time_threshold)
	{
		if(!data_map.empty())
		{
			auto larger = data_map.lower_bound(time);
			auto smaller = larger;
			if(larger != data_map.begin())
			{
				smaller--;
			}

			double diff_larger = abs(larger->first - time);
			double diff_smaller = abs(smaller->first - time);

			if(std::min(diff_larger, diff_smaller) > time_threshold)
			{ return nullptr; }

			if(diff_larger < diff_smaller)
			{
				return &larger->second;
			}
			else{ return &smaller->second; }
		}
		return nullptr;
	}







	std::map<double, std::string> depths_paths_;
	std::map<double, std::string> color_paths_;
	std::map<double, Eigen::Affine3f> poses_;

	std::map<double, std::string> input_depth_paths;

	bool is_obj_;
	bool is_img_;
};

} /* namespace tangodata */

#endif /* TANGODATA_H_ */
