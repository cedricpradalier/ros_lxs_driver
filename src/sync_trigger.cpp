
#include <ros/ros.h>
#include <std_msgs/Header.h>


class SyncTrigger {
    protected:
        int num_publisher;
        double rate;
        std::vector<ros::Publisher> publishers;
        ros::Timer timer;

        size_t next;

        void triggerCallback(const ros::TimerEvent&) {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            publishers[next].publish(header);
            next = (next + 1) % publishers.size();
        }

    public:
        SyncTrigger(ros::NodeHandle & nh) {
            next = 0;
            nh.param("num_publisher",num_publisher,2);
            nh.param("rate",rate,100.0);
            num_publisher = std::max(1,num_publisher);

            for (int i=0;i<num_publisher;i++) {
                char name[128];
                sprintf(name,"trigger%d",i);
                publishers.push_back(nh.advertise<std_msgs::Header>(name,1));
            }
            timer = nh.createTimer(ros::Duration(1./rate), &SyncTrigger::triggerCallback,this);
        }

};

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"sync_trigger");
    ros::NodeHandle nh("~");
    SyncTrigger driver(nh);

    ros::spin();

    return 0;
}
