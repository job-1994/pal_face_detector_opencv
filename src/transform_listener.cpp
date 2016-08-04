    #include <ros/ros.h>
    #include <tf/transform_listener.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/Image.h>
    #include <sensor_msgs/image_encodings.h>

    class TransformListener
    {
    public:
        TransformListener(ros::NodeHandle nh_);
        ~TransformListener();
        ros::Publisher vel_pub;
        ros::Subscriber depth_sub;
        image_transport::ImageTransport ImageTransport_;
        image_transport::Subscriber image_sub;
        void loop();
        void imageCB(const sensor_msgs::ImageConstPtr& msg);
        void pclCB(const sensor_msgs::PointCloudConstPtr& pcl);
    protected:
        tf::TransformListener listener;
        void pubVel(float z);
    };



    TransformListener::TransformListener(ros::NodeHandle nh_) : ImageTransport_(nh_)
    {
        ROS_INFO("Subscribing");
        image_sub = ImageTransport_.subscribe("/xtion/depth_registered/image_rect", 1, &TransformListener::imageCB, this, image_transport::TransportHints("compressedDepth"));
        depth_sub = nh_.subscribe("xtion/depth_registered/points", 1, &TransformListener::pclCB, this);
        ROS_INFO("Subscribed");
        vel_pub = nh_.advertise<geometry_msgs::Twist>("nav_vel", 1);
    }

    TransformListener::~TransformListener(){}

    void TransformListener::pclCB(const sensor_msgs::PointCloudConstPtr& pcl)
    {
        ROS_INFO("Reached");
    }

    void TransformListener::imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
        int h = msg->height;
        int w = msg->width;
        ROS_INFO_STREAM("Height, Width: " << h << ", " << w);
    }

    void TransformListener::loop()
    {
        tf::Vector3 translations_head, transl;
        tf::StampedTransform transform_head;
        try
        {
            //transform of head to base
            listener.lookupTransform("/head_1_link", "/base_link", ros::Time(), transform_head);

            //transform of original base position to current
//            listener.lookupTransform();
        }
        catch(tf::TransformException ex){
//            ROS_ERROR("%s", ex.what());
        }

        translations_head = transform_head.getOrigin();
//        ROS_INFO_STREAM("X: " << translations_head.getX() << ", Y: " << translations_head.getY() << ", Z: " << translations_head.getZ());
    }

    void TransformListener::pubVel(float z)
    {
        geometry_msgs::Twist vel;
        vel.angular.x = vel.angular.y = 0;
        vel.linear.x = vel.linear.y = vel.linear.z = 0;
        vel.angular.z = z;

    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "transform_listener");

        ros::NodeHandle nh;

        TransformListener tf(nh);

//        while(ros::ok())
//            tf.loop();

         ros::spin();

        return 0;
    }
