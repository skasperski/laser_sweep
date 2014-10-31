#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <iostream>

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, "Sweep");
	NodeHandle n;

	tf::TransformListener tfListener;
	tf::StampedTransform tf1, tf2;
	
	Rate r(10);
	while(ok())
	{
		Time now = Time::now();
		if(!tfListener.waitForTransform("laser_link", "camera", now, Duration(3.0)))
			continue;
		if(!tfListener.waitForTransform("camera_init_2", "laser_link", now, Duration(3.0)))
			continue;

		try
		{
			tfListener.lookupTransform("laser_link", "camera", now, tf1);
			tfListener.lookupTransform("camera_init_2", "laser_link", now, tf2);
			tf::Transform tf3 = tf1 * tf2;
			ROS_INFO("TF: %.2f, %.2f, %.2f / %.2f, %.2f, %.2f, %.2f",
					tf3.getOrigin()[0],
					tf3.getOrigin()[1],
					tf3.getOrigin()[2],
					tf3.getRotation().getAxis()[0],
					tf3.getRotation().getAxis()[1],
					tf3.getRotation().getAxis()[2],
					tf3.getRotation().getW());
		}catch(tf::TransformException e)
		{
			ROS_ERROR("%s", e.what());
		}

		spinOnce();
		r.sleep();
	}

	return 0;
}
