/*
 * Point3D.h
 *
 *  Created on: 2017年1月3日
 *      Author: tomzhang
 */

#ifndef INCLUDE_POINTCLOUD_SEGMENTATION_POINT3D_H_
#define INCLUDE_POINTCLOUD_SEGMENTATION_POINT3D_H_


#include<cmath>
namespace pointcloud_segmentation {
class Point3D
{
public:
	enum UNEVENNESS{
		UNKNOW , EVEN, UNEVEN
	};
	Point3D( float x, float y, float z);
	~Point3D();
	float X();
	float Y();
	float Z();
	//float unevenness();
	//float &virtuel_range();
	float index();
	void set_index(int idx);
	bool passable_;
	float x_, y_,z_;
	int index_;
	float virtuel_range_,virtuel_h;
	UNEVENNESS unevenness_;
	inline float static euclidean_distance(const Point3D &p1, const Point3D &p2){
		return std::sqrt((float)(std::pow(p1.x_-p2.x_,2)+std::pow(p1.z_-p2.z_,2)));//+std::pow(p1.y_-p2.y_,2)
	}


private:
};
}
#endif /* INCLUDE_POINTCLOUD_SEGMENTATION_POINT3D_H_ */


