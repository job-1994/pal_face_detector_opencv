    #include <ros/ros.h>
    #include <tf/transform_listener.h>


    class TransformListener
    {
    public:
        TransformListener();
        ~TransformListener();
        void transformCB();
    protected:
        tf::TransformListener listener;
    };



    TransformListener::TransformListener(){ }

    TransformListener::~TransformListener(){}

    void TransformListener::transformCB()
    {
        tf::Vector3 translations_head;
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
        ROS_INFO_STREAM("X: " << translations_head.getX() << ", Y: " << translations_head.getY() << ", Z: " << translations_head.getZ());
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "transform_listener");

        TransformListener tf;

        while(ros::ok())
            tf.transformCB();

         ros::spin();

        return 0;
    }
