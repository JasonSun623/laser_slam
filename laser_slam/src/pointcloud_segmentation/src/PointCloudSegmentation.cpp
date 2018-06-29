#include "ros/ros.h"
#include "pointcloud_segmentation/PointCloudSegmentation.h"
using namespace pointcloud_segmentation;
PointCloudSegmentation::PointCloudSegmentation(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int rows,
		const int cols, const float camera_x, const float camera_y,
		const float camera_z, const float camera_view, const float camera_pitch,
		const float obstacle_height_threshold, const float dist_threshold,
		const float slope_threshold) :
				cloud_(cloud), rows_(rows), cols_(cols), camera_point_(camera_x,
						camera_y, camera_z), camera_view_(camera_view), camera_pitch_(camera_pitch),obstacle_height_threshold_(
								obstacle_height_threshold), organizator_(cloud, rows, cols), dist_threshold_(
										dist_threshold), slope_threshold_(
												M_PI * slope_threshold / 180.0)
{
}
PointCloudSegmentation::~PointCloudSegmentation()
{

}
void PointCloudSegmentation::segmentation(
		pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud,
		pcl::PointCloud<pcl::PointXYZ> &ground_cloud)
{

	//calculate_angle();
	calculate_unevenness();

	point_segment(obstacle_cloud, ground_cloud);
}
inline void PointCloudSegmentation::calculate_delta_r(const float r,
		const float h, const float delta_pitch, float &delta_r)
{

	delta_r = r * std::sqrt(r * r - h * h) * delta_pitch / h;
	//ROS_INFO("%f,%f,%f,%f",r,h,delta_pitch,delta_r);
}
inline void PointCloudSegmentation::calculate_delta_r_threshold(const float r,
		const float h, const float angle_threshold, const float delta_pitch,
		float &delta_r)
{
	float angle = angle_threshold + std::asin(h / r);
	float h_th = r * std::sin(angle);
	delta_r = r * std::sqrt(r * r - h_th * h_th) * delta_pitch / h_th;

}
void PointCloudSegmentation::calculate_unevenness()
{

	float delta_r_sensor = 0.0;

	Point3D *inner = NULL;
	Point3D *outer = NULL;
	float h = (camera_point_.z_);

	float r_inner = 0.0;
	float r_outer = 0.0;
	bool height_test = false;
	int start_idx = -1, end_idx = -1;
	float obstacle_height = 0.0, ground_height = 0.0;
	//float camera_view = this->camera_view_ /360.0 * M_PI;
	float delta_pitch = this->camera_view_ / 180.0 * M_PI / (rows_ - 1);
	//float delta_pitch = 0;
	float delta_r_expected = 0.0;
	float delta_r_threshold = 0.0;
	float delta_r_threshold_gound = 0.0;
	float theta = 0.0;
	float omega_threshold = 0.0;

	//Point3D abs_ground_point(camera_point_.z_/std::tan(M_PI/2+camera_pitch),camera_point_.y_,0.0);
	float virtuel_range = 0.0;
	float ground_h = 0.0;
    bool ground_point_found = false;
	/*
	 for(int j = 0; j<cols_;j++){
	 inner = const_cast<Point3D*>(&organizator_.points[j][rows_ - 1]);
	 if (!std::isnan(inner->z_)) {
	 //ROS_INFO("point h = %f", inner->z_);
	 if(inner->z_ < ground_h+0.05&&inner->z_>ground_h-0.05 ){// the point inner is a ground point
	 inner->virtuel_range_ = Point3D::euclidean_distance(camera_point_, *inner);
	 inner->virtuel_h= camera_point_.z_-inner->z_;
	 }
	 else
	 {
	 inner->virtuel_range_ = virtuel_range;
	 inner->virtuel_h = camera_point_.z_ - ground_h;
	 }

	 }
	 }*/

	//  find the ground height from points in 1st row
	for (int j = 0; j < cols_; j++)
	{
		inner = const_cast<Point3D*>(&organizator_.points[j][rows_ - 1]);
		//ROS_INFO("%f,%f,%f,%f",inner->x_,inner->y_,inner->z_,organizator_.points[rows_-1][j].virtuel_range_);
		//ROS_INFO("%d,%f", j, inner->z_);
		if (!std::isnan(inner->z_) && inner->z_ < 0.06 && inner->z_ > -0.06)
		{
			ground_h = inner->z_;
			virtuel_range = Point3D::euclidean_distance(camera_point_, *inner);
			//ROS_INFO(" inner_z:%f",inner->z_);
			//ROS_INFO(" h:%f, r:%f", ground_h, virtuel_range);
			ground_point_found=true;
		}
		if(!ground_point_found){
			virtuel_range = h/std::cos((80.0-camera_pitch_-camera_view_/2.0) / 180.0 * M_PI );
			//ROS_INFO(" virtuel_range:%f",virtuel_range);
		}
	}
	//
	for (int j = 0; j < cols_; j++)
	{
		inner = const_cast<Point3D*>(&organizator_.points[j][rows_ - 1]);

		if (!std::isnan(inner->z_ )&&inner->z_ <  0.06)
		{
			inner->virtuel_range_ = Point3D::euclidean_distance(camera_point_,
					*inner);
			inner->virtuel_h = h - inner->z_;
		}
		else
		{
			inner->virtuel_range_ = virtuel_range;
			inner->virtuel_h = h - ground_h;
			//ROS_INFO(" r_inner:%f, r_outer:%f",inner->virtuel_range_,inner->virtuel_h);
		}
	}

	for (int j = 0; j < cols_; j++)
	{
		ground_height=0.0;
		//for each point in vertical direction, calculate its unevenness value
		for (int i = rows_ - 1; i >= 2; i--)
		{

			inner = const_cast<Point3D*>(&organizator_.points[j][i]);
			outer = const_cast<Point3D*>(&organizator_.points[j][i - 1]);

			r_inner = inner->virtuel_range_;
			h = inner->virtuel_h;
			// TODO find ground height


			calculate_delta_r_threshold(r_inner, h, slope_threshold_/2.0,
					delta_pitch, delta_r_threshold_gound);
			//ROS_INFO("delta_r_expected:%f",delta_r_expected);
			if (std::isnan(outer->x_))
			{

				//outer->unevenness_=Point3D::UNKNOW;
				virtuel_range = r_inner + delta_r_threshold_gound;
				h -= delta_r_threshold_gound * std::tan(slope_threshold_/2.0);


			}
			else
			{
				// calculate the expected diff range of adjacent points in vertical direction
				calculate_delta_r_threshold(r_inner, h, slope_threshold_,
						delta_pitch, delta_r_threshold);
				// calculate the range between camera and points
				r_outer = Point3D::euclidean_distance(camera_point_, *outer);

				// calculate the diff range of adjacent points in vertical direction by sensor

				delta_r_sensor = r_outer - r_inner;

				// calculate unevenness theta
				//theta = 1 - delta_r_sensor / delta_r_expected;
				//std::isnan(inner->x_)&&
				omega_threshold = 1 - delta_r_sensor / delta_r_threshold;

				if (omega_threshold <= 0)
				{ //the angle of point on slope is passable
					outer->unevenness_=Point3D::EVEN;
					if (height_test) // determine whether the points before are obstacle if heigth_test was started
					{
						height_test = false;
						obstacle_height_test( obstacle_height, start_idx, i, j);
					}
					ground_height = outer->z_;
					virtuel_range = r_outer;
					h = camera_point_.z_ - outer->z_;
				}
				else
				{ //the angle of point on slope can not be pass, then test the height of obstacle points
					outer->unevenness_ = Point3D::UNEVEN;


					if (!height_test)
					{
						start_idx = i - 1;
						height_test = true;
						obstacle_height=0.0;
					}
//					else
//					{
//						float dist_inner_and_outer =10.0;
//						if(inner->unevenness_!=Point3D::UNKNOW){
//							dist_inner_and_outer =Point3D::euclidean_distance(*inner, *outer);
//						}
//						if (dist_inner_and_outer > dist_threshold_)
//						{
//							if (obstacle_height < obstacle_height_threshold_)
//							{
//								for (int l = start_idx; l >= i; l--)
//								{
//									inner =
//											const_cast<Point3D*>(&organizator_.points[j][l]);
//									inner->unevenness_=Point3D::EVEN;
//								}
//								obstacle_height = 0.0;
//								start_idx = i - 1;
//							}
//
//						}
//					}
					if (obstacle_height < outer->z_ - ground_height)
					{
						obstacle_height = outer->z_ - ground_height;
					}

					virtuel_range = r_inner + delta_r_threshold_gound;
					h -= delta_r_threshold_gound * std::tan(slope_threshold_/2.0);
				}
			}
			outer->virtuel_range_ = virtuel_range;
			outer->virtuel_h = h;
		}
		height_test = false;
	}

}

void PointCloudSegmentation::calculate_angle()
{
	Point3D* pointA;
	Point3D* pointB;
	float a, b, c, angle;
	bool flag = true;
	int ii, jj;
	for (int j = 0; j < cols_; j++)
	{
		//for(int i=rows_-1;i>=0;i--){
		pointA = const_cast<Point3D*>(&organizator_.points[379][j]);
		pointB = const_cast<Point3D*>(&organizator_.points[1][j]);
		//	if(!std::isnan(pointA->x_)){
		//		if(flag){
		//			ROS_INFO("last:%d,%d.",i,j);
		//			flag =false;
		//		}
		//		ii=i;jj=j;
		//	}
		if (std::isnan(pointA->x_) || std::isnan(pointB->x_))
		{
			//			ROS_INFO("%d,%d.",i,j);
			continue;
		}
		a = Point3D::euclidean_distance(*pointB, camera_point_);
		b = Point3D::euclidean_distance(*pointA, camera_point_);
		c = Point3D::euclidean_distance(*pointA, *pointB);
		angle = std::acos((a * a + b * b - c * c) / (2 * a * b));
		ROS_INFO("a:%f,b:%f,c:%f,%d anlge is:%f degree.", a, b, c, j,
				angle/M_PI*180.0);
		//}
		//ROS_INFO("first:%d,%d.",ii,jj);
		//flag = true;
	}

}
void PointCloudSegmentation::visualization(visualization_msgs::Marker &points_g,
		visualization_msgs::Marker &points_o)
{

	points_g.ns = "points_g";
	points_o.ns = "points_o";
	points_g.action = points_o.action = visualization_msgs::Marker::ADD;
	points_g.pose.orientation.w = points_o.pose.orientation.w = 1.0;
	points_g.id = 0;
	points_o.id = 1;
	points_g.type = points_o.type = visualization_msgs::Marker::POINTS;
	points_g.scale.x = points_o.scale.x = 0.005;
	points_g.scale.y = points_o.scale.y = 0.005;
	points_g.color.g = 1.0f;
	points_g.color.a = 1.0;
	points_o.color.r = 1.0f;
	points_o.color.a = 1.0;
	for (int j = 0; j < cols_; j++)
	{
		for (int i = 0; i < rows_ - 1; i++)
		{

			if (std::isnan(organizator_.points[j][i].x_))
			{
				continue;
			}
			else
			{
				geometry_msgs::Point p;
				p.x = organizator_.points[j][i].x_;
				p.y = organizator_.points[j][i].y_;
				p.z = organizator_.points[j][i].z_;
				if (organizator_.points[j][i].unevenness_==Point3D::EVEN)
				{
					points_g.points.push_back(p);
				}
				else if(organizator_.points[j][i].unevenness_==Point3D::UNEVEN)
				{
					points_o.points.push_back(p);

				}
			}
		}
	}
	//ROS_INFO("os:%d,gs:%d",(int)( points_o.points.size()),(int) (points_g.points.size()));

}

inline void PointCloudSegmentation::obstacle_height_test( float &height, int &start_idx, int end_idx, const int col_idx){
	// determine whether the points before are obstacle if heigth_test was started

		Point3D* point;
		if (height < obstacle_height_threshold_)
		{
			for (int i = start_idx; i >= end_idx; i--)
			{
				point = const_cast<Point3D*>(&organizator_.points[col_idx][i]);
				if(point->unevenness_==Point3D::UNEVEN)
				{
							point->unevenness_=Point3D::EVEN;
				}
			}
		}

}

void PointCloudSegmentation::point_segment(
		pcl::PointCloud<pcl::PointXYZ>&obstacle_cloud,
		pcl::PointCloud<pcl::PointXYZ> &ground_cloud)
{
	obstacle_cloud.points.resize(rows_ * cols_);
	ground_cloud.points.resize(rows_ * cols_);
	unsigned int obstacle_point_count = 0, ground_point_count = 0;
	for (int j = 0; j < cols_; j++)
	{
		for (int i = 0; i < rows_; i++)
		{

			if (!std::isnan(organizator_.points[j][i].x_))
			{
				if (organizator_.points[j][i].unevenness_==Point3D::UNEVEN)
				{
					obstacle_cloud.points[obstacle_point_count++] =
							cloud_->points[i * cols_ + j];
					//ROS_DEBUG();
				}
				else if(organizator_.points[j][i].unevenness_==Point3D::EVEN)
				{
					ground_cloud.points[ground_point_count++] = cloud_->points[i* cols_ + j];
				}
			}
		}
	}
}
