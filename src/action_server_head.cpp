#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>

class HeadServer
{
public:
	HeadServer(ros::NodeHandle nh_);
	~HeadServer();
	ros::Subscriber data_sub;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction>* PointHeadActionClient;
	geometry_msgs::PointStamped pointStamped;
	ros::ServiceServer service;

protected:
    void createAction();
	void dataCB(const geometry_msgs::PointStamped& msg);
    bool call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

HeadServer::HeadServer(ros::NodeHandle nh_)
{
	ROS_INFO("Waiting for Action Server");
    PointHeadActionClient = new actionlib::SimpleActionClient<control_msgs::PointHeadAction>("/head_controller/point_head_action", true);
	ROS_INFO("Connected to Action Server");
    service = nh_.advertiseService("find_head", &HeadServer::call, this);
	data_sub = nh_.subscribe("pal_face/data", 1, &HeadServer::dataCB, this);
}

HeadServer::~HeadServer(){}

void HeadServer::dataCB(const geometry_msgs::PointStamped& msg)
{
	
	// this->createAction(msg);
	pointStamped = msg;
}


bool HeadServer::call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Starting head movement upwards");
    this->createAction();
    ROS_INFO("Finding face");
    this->createAction();


    return true;
}

void HeadServer::createAction()
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
      ROS_INFO("Finished moving head");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HeadServerClient");
	ros::NodeHandle nh;
	HeadServer ha(nh);
	ros::spin();
	return 0;
}

