#include "pointcloud_segmentation/Point3D.h"
using namespace pointcloud_segmentation;
Point3D::Point3D( float x, float y, float z) :
		passable_(true),index_(0), x_(x), y_(y), z_(z), unevenness_(UNKNOW), virtuel_range_(0.0),virtuel_h(0.0)
{


}
Point3D::~Point3D()
{
}
float Point3D::X()
{
	return x_;
}
float Point3D::Y()
{
	return y_;
}
float Point3D::Z()
{
	return z_;
}
float Point3D::index(){
	return index_;
}
void Point3D::set_index(int idx){
	index_=idx;
}

