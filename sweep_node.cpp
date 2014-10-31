#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define ID 1
#define PI 1.570796327
#define CORR -0.15

using namespace ros;

bool up;
Publisher* gCommandPublisher;
tf::TransformBroadcaster* gTfBroadcaster;


const double min = -PI + CORR;
const double max = PI + CORR;

void statusHandler(const dynamixel_msgs::JointStateConstPtr& jointState)
{
	// Send Transform
	double angle = jointState->current_pos - CORR;
	tf::Transform transform(tf::createQuaternionFromRPY(angle, 0, 0));
	gTfBroadcaster->sendTransform(tf::StampedTransform(transform, jointState->header.stamp, "dxl_base", "dxl_rotor"));

	// Send command
	if(!up && jointState->current_pos < min)
	{
		std_msgs::Float64 cmd;
		cmd.data = max + 0.1;
		gCommandPublisher->publish(cmd);
		up = true;
	}else if(up && jointState->current_pos > max)
	{
		std_msgs::Float64 cmd;
		cmd.data = min - 0.1;
		gCommandPublisher->publish(cmd);
		up = false;
	}
}

int main(int argc, char **argv)
{
	init(argc, argv, "Sweep");
	NodeHandle n;
	
	tf::TransformBroadcaster tfbc;
	gTfBroadcaster = &tfbc;

	Subscriber statusSubscriber = n.subscribe("status", 10, statusHandler);
	Publisher commandPublisher = n.advertise<std_msgs::Float64>("command", 10);
	gCommandPublisher = &commandPublisher;

	Rate r(1);
	while(commandPublisher.getNumSubscribers() == 0)
	{
		r.sleep();
	}

	std_msgs::Float64 cmd;
	cmd.data = max + 0.1;
	commandPublisher.publish(cmd);
	up = true;
	
	spin();

	return 0;
}
