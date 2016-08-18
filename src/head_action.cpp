#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>

class HeadAction
{
public:
	HeadAction(ros::NodeHandle nh_);
	~HeadAction();
	ros::Subscriber data_sub;
	actionlib::SimpleActionClient<control_msgs::PointHeadAction>* PointHeadActionClient;


protected:
	void createAction(geometry_msgs::PointStamped pointStamped);
	void dataCB(const geometry_msgs::PointStamped& msg);
};

HeadAction::HeadAction(ros::NodeHandle nh_)
{
	ROS_INFO("Waiting for Action Server");
	PointHeadActionClient = new actionlib::SimpleActionClient<control_msgs::PointHeadAction>("/head_controller/point_head_action", true);
	ROS_INFO("Connected to Action Server");

	data_sub = nh_.subscribe("pal_face/data", 1, &HeadAction::dataCB, this);
}

HeadAction::~HeadAction(){}

void HeadAction::dataCB(const geometry_msgs::PointStamped& msg)
{
	
	this->createAction(msg);
}

void HeadAction::createAction(geometry_msgs::PointStamped pointStamped)
{
      control_msgs::PointHeadGoal goal;
      goal.pointing_frame = "/xtion_rgb_optical_frame";
      goal.pointing_axis.x = 0.0;
      goal.pointing_axis.y = 0.0;
      goal.pointing_axis.z = 1.0;
      goal.min_duration = ros::Duration(1.0);
      goal.max_velocity = 0.25;
      goal.target = pointStamped;
      PointHeadActionClient->sendGoal(goal);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HeadActionClient");
	ros::NodeHandle nh;
	HeadAction ha(nh);
	ros::spin();
	return 0;
}

