/*
 * utils.h
 *
 *  Created on: Apr 17, 2016
 *      Author: qkgautier
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <sstream>
#include <iomanip>


//********************************************
// Types
//********************************************

namespace tangodata
{

enum DepthInterpolationType
{
	DEPTH_INTERP_NONE = 0,
	DEPTH_INTERP_NN,
	DEPTH_INTERP_BILINEAR
};

struct Intrinsics
{
	uint32_t width;
	uint32_t height;
	double fx, fy, cx, cy;

	Intrinsics(uint32_t w=0, uint32_t h=0, double afx=0.0, double afy=0.0, double acx=0.0, double acy=0.0):
		width(w), height(h), fx(afx), fy(afy), cx(acx), cy(acy){}

	Intrinsics scaled(double factor)
	{ return Intrinsics(width*factor, height*factor, fx*factor, fy*factor, cx*factor, cy*factor); }

};

}  // namespace tangodata



//********************************************
// Functions
//********************************************

namespace tangodata {

//-------------------------------------------------------
inline bool has_suffix(const std::string& str, const std::string& suffix)
//-------------------------------------------------------
{
    if(str.size() >= suffix.size())
    { return str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0; }
    else{ return false; }
}

//-------------------------------------------------------
template <typename T>
std::string to_str(const T value, const int precision = 6, bool fixed = true)
//-------------------------------------------------------
{
    std::ostringstream out;
    if(fixed) { out << std::fixed; }
    out << std::setprecision(precision) << value;
    return out.str();
}



}  // namespace tangodata




#endif /* UTILS_H_ */
