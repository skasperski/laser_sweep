#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


tf::TransformListener* gTfListener;
laser_geometry::LaserProjection* gProjector;
ros::Publisher* gCloudPublisher;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::PointCloud2 cloud;
	gTfListener->waitForTransform(scan->header.frame_id, "camera", scan->header.stamp, ros::Duration(0.1));
	try
	{
		gProjector->transformLaserScanToPointCloud("camera", *scan, cloud, *gTfListener);
	}catch(tf::TransformException)
	{
		return;
	}
	
	unsigned int x_off = cloud.fields[0].offset;
	unsigned int y_off = cloud.fields[1].offset;
	unsigned int z_off = cloud.fields[2].offset;

	cloud.fields[0].offset = y_off;
	cloud.fields[1].offset = z_off;
	cloud.fields[2].offset = x_off;

/*
	// Switch coordinates according to LOAM
	unsigned int pclSize = cloud.width * cloud.height;
	pcl::PointXYZ p;
	for(unsigned int i; i < pclSize; i++)
	{
		unsigned int index i * cloud.point_step;
		p = cloud.points[i];
		cloud.points[i].x = p.y;
		cloud.points[i].y = p.z;
		cloud.points[i].z = p.x;
	}
*/
	gCloudPublisher->publish(cloud);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan2pcl");

	ros::NodeHandle n;
	tf::TransformListener tfl(n);
//	tfl.setExtrapolationLimit(ros::Duration(0.1));
	gTfListener = &tfl;

	laser_geometry::LaserProjection projector;
	gProjector = &projector;

	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &scanCallback);
	ros::Publisher cloudPub = n.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
	gCloudPublisher = &cloudPub;

	ros::spin();

	return 0;
}
