#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


tf::TransformListener* gTfListener;
laser_geometry::LaserProjection* gProjector;
ros::Publisher* gCloudPublisher;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::PointCloud2 cloud;
	std::string error_msg;
	if(!gTfListener->waitForTransform(scan->header.frame_id,
	                                  "camera",
	                                  scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
	                                  ros::Duration(2.0),
	                                  ros::Duration(0.01),
	                                  &error_msg))
	{
		ROS_WARN("%s", error_msg.c_str());
		return;
	}
	try
	{
		gProjector->transformLaserScanToPointCloud("camera", *scan, cloud, *gTfListener);
	}catch(tf::TransformException e)
	{
		ROS_WARN("Could not transform scan to pointcloud! (%s)", e.what());
		return;
	}
	
	unsigned int x_off = cloud.fields[0].offset;
	unsigned int y_off = cloud.fields[1].offset;
	unsigned int z_off = cloud.fields[2].offset;

	cloud.fields[0].offset = y_off;
	cloud.fields[1].offset = z_off;
	cloud.fields[2].offset = x_off;

	gCloudPublisher->publish(cloud);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan2pcl");

	ros::NodeHandle n;
	tf::TransformListener tfl(n);
	gTfListener = &tfl;

	laser_geometry::LaserProjection projector;
	gProjector = &projector;

	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &scanCallback);
	ros::Publisher cloudPub = n.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
	gCloudPublisher = &cloudPub;

	ros::spin();

	return 0;
}
