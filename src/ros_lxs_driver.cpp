
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <set>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>

#include "LxS_Lib/LxS_Datagram.h"

using boost::asio::ip::udp;
using namespace LxS;

class LxSDriver {
    protected:
        bool connected;
        ros::Publisher scan_pub;
        ros::Timer timer;
        std::string sensor_host;
        std::string sensor_port;
        int host_port;
        std::string base_frame;
        double wait_timeout;
        double rate_control;
        bool trigger_input;
        bool cascade_output;
        int debug;
        unsigned int pkt_num;
        bool activate_trigger;
        static boost::asio::io_service io_service;

        boost::shared_ptr<udp::endpoint> receiver_endpoint;
        boost::shared_ptr<udp::socket> send_socket;

        template <class iterator> 
            void dump_buf(iterator begin, const iterator & end, const std::string & prefix) {
                printf("%s ",prefix.c_str());
                while (begin != end) {
                    printf("%02X ",*begin);
                    begin ++;
                }
                printf("\n");
            }

        // pthread_mutex_t waitingListMtx = PTHREAD_MUTEX_INITIALIZER;
        // pthread_cond_t waitingListCond = PTHREAD_COND_INITIALIZER;
        std::set<eCmd> waitingList;
        LxS::DatagramPtr response;

        std::thread* receiveThread;
        std::condition_variable receiveCond;
        std::mutex receiveMtx;

        template <class datagram>
            void handleCmdDatagram(const datagram & dg) {
                std::unique_lock<std::mutex> lk(receiveMtx);
                std::set<eCmd>::iterator it = waitingList.find(dg.cmd);
                if (it != waitingList.end()) {
                    response = convertToInheritedDatagram(dg);
                    waitingList.clear();
                    receiveCond.notify_all();
                }
            }

        void publishPointCloud(const RespDataX & dx, const RespDataZ & dz) {
            if (!connected) return;
            if (debug > 1) {
                printf("P");fflush(stdout);
            }
            pcl::PointCloud<pcl::PointXYZ> pc;
            const std::vector<float> & X = dx.getX();
            const std::vector<float> & Z = dz.getZ();
            assert(X.size() == Z.size());
            pc.resize(X.size());
            for (size_t i=0;i<X.size();i++) {
                pc.push_back(pcl::PointXYZ(X[i],0,Z[i]));
            }
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(pc,msg);
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id = base_frame;
            scan_pub.publish(msg);
        }

        void publishPointCloud(const RespZXCoordinates & dxz) {
            if (!connected) return;
            if (debug > 1) {
                printf("Q");fflush(stdout);
            }
            pcl::PointCloud<pcl::PointXYZ> pc;
            const std::vector<float> & X = dxz.getX();
            const std::vector<float> & Z = dxz.getZ();
            assert(X.size() == Z.size());
            pc.resize(X.size());
            for (size_t i=0;i<X.size();i++) {
                pc.push_back(pcl::PointXYZ(X[i],0,Z[i]));
            }
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(pc,msg);
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id = base_frame;
            scan_pub.publish(msg);
        }

        void recv_thread() {
            ROS_INFO("%s: Listening on port %d",sensor_host.c_str(),host_port);
            udp::socket socket(io_service, udp::endpoint(udp::v4(), host_port));
            uint8_t axis = 0;
            RespDataX dx;
            RespDataZ dz;
            RespZXCoordinates dxz;
            while (ros::ok()) {
                boost::array<uint8_t, 1024> recv_buf;
                udp::endpoint sender_endpoint;
                int len = socket.receive_from(
                        boost::asio::buffer(recv_buf), sender_endpoint);
                if (len>0) {

                    Datagram dg;
                    dg.deserialize(recv_buf.begin(),recv_buf.end());

                    switch (dg.cmd) {
                        case LXS_RESP_DATA_Z:
                            {
                                // printf("Z");fflush(stdout);
                                dz = RespDataZ(dg);
                                axis |= 1;
                                if (axis == 3) {
                                    publishPointCloud(dx,dz);
                                    axis = 0;
                                }
                            }
                            break;
                        case LXS_RESP_DATA_X:
                            {
                                // printf("X");fflush(stdout);
                                dx = RespDataX(dg);
                                axis |= 2;
                                if (axis == 3) {
                                    publishPointCloud(dx,dz);
                                    axis = 0;
                                }
                            }
                            break;
                        case LXS_RESP_XZ_HIGH_DATA:
                            {
                                // printf("I");fflush(stdout);
                                dxz = RespZXCoordinates(dg);
                                publishPointCloud(dxz);
                            }
                            break;
                        default:
                            if (debug>0) {
                                printf("\n%s Recv: ",sensor_host.c_str());
                                dg.dump();
                                dump_buf(recv_buf.begin(),recv_buf.begin()+len,"");
                            }
                            handleCmdDatagram(dg);

                            break;
                    }
                } else {
                    usleep(1000);
                }
            }
        }

        template <class datagram>
            int send(const datagram & dg) {
                std::vector<uint8_t> send_buf(dg.serialized_length());
                dg.serialize(send_buf.begin(),send_buf.end());
                if (debug) {
                    dg.dump();
                    dump_buf(send_buf.begin(),send_buf.end(),sensor_host);
                }

                return send_socket->send_to(boost::asio::buffer(send_buf), *receiver_endpoint);
            }

        template <class datagram>
            bool send_and_wait(const datagram & dg, unsigned int num_retries=0) {
                std::vector<uint8_t> send_buf(dg.serialized_length());
                dg.serialize(send_buf.begin(),send_buf.end());
                if (debug) {
                    dg.dump();
                    dump_buf(send_buf.begin(),send_buf.end(),sensor_host);
                }
                eCmd resp_cmd = eCmdResponse(dg.cmd);

                int ntrials = num_retries + 1;
                for (int i=0;i<ntrials;i++) {
                    std::unique_lock<std::mutex> lk(receiveMtx);
                    waitingList.insert(LxS::LXS_RESP_ACK_FAILURE);
                    waitingList.insert(resp_cmd);
                    send_socket->send_to(boost::asio::buffer(send_buf), *receiver_endpoint);
                    std::chrono::duration<double> timeout(wait_timeout);
                    std::cv_status wstatus = receiveCond.wait_for(lk, timeout);
                    waitingList.clear();
                    bool retrying = i < (ntrials-1);
                    if (wstatus == std::cv_status::timeout) {
                        ROS_ERROR("%s: Timeout while waiting for answer to %04X (%s) %s",sensor_host.c_str(),uint16_t(dg.cmd),eCmdString(dg.cmd),retrying?" - Retrying":"");
                    } else if (response) {
                        switch (response->cmd) {
                            case LXS_RESP_ACK_FAILURE:
                                ROS_ERROR("%s: Failure as answer to %04X (%s): %04X %s",sensor_host.c_str(),uint16_t(dg.cmd),eCmdString(dg.cmd),dg.status,retrying?" - Retrying":"");
                                break;
                            case LXS_RESP_ACK_SUCCESS:
                            default:
                                return true;
                        }
                    }
                }
                return false;
            }

        void triggerCallback(const ros::TimerEvent&) {
            if (!activate_trigger) {
                return;
            }
            CmdEthernetTrigger trigger(pkt_num++);
            send(trigger);
        }

    public: 
        LxSDriver(ros::NodeHandle & nh) {
            connected = false;
            pkt_num = 1;
            activate_trigger = false;
            nh.param("base_frame",base_frame,std::string("lrs36"));
            nh.param("sensor_host",sensor_host,std::string("192.168.60.3"));
            nh.param("sensor_port",sensor_port,std::string("9008"));
            nh.param("host_port",host_port,5634);
            nh.param("wait_timeout",wait_timeout,0.5);
            nh.param("trigger_input",trigger_input,false);
            nh.param("cascade_output",cascade_output,false);
            nh.param("rate_control",rate_control,-1.0);
            if (rate_control > 100) {
                ROS_WARN("Rate control (%f) cannot be larger than 100",rate_control);
                rate_control = 100;
            }
            nh.param("debug",debug,0);
            scan_pub = nh.advertise<sensor_msgs::PointCloud2>("scans",1);
            if (rate_control>0) {
                timer = nh.createTimer(ros::Duration(1./rate_control), &LxSDriver::triggerCallback,this);
            }
        }

       void run() {
           udp::resolver resolver(io_service);
           udp::resolver::query query(udp::v4(), sensor_host,sensor_port);
           receiver_endpoint.reset(new udp::endpoint(*resolver.resolve(query)));


           send_socket.reset(new udp::socket(io_service));
           send_socket->open(udp::v4());

           receiveThread = new std::thread(&LxSDriver::recv_thread,this);

           pkt_num = 1;
           bool res = false;

           connected = false;

           try {
               while (!connected && ros::ok()) {
                   ros::spinOnce();
                   ROS_INFO("Connecting to %s:%s",sensor_host.c_str(),sensor_port.c_str());
                   CmdConnectToSensor connect(pkt_num++);
                   res = send_and_wait(connect,4);
                   if (res) {
                       connected = true;
                       ROS_INFO("Connected to %s",sensor_host.c_str());
                       break;
                   }
                   CmdDisconnectFromSensor disconnect(pkt_num++);
                   res = send_and_wait(disconnect);
                   ros::Duration(1).sleep();
               }
               if (!connected && !ros::ok()) {
                   return;
               }
               CmdEnterCommandMode enter(pkt_num++);
               res = send_and_wait(enter,4);
               if (res) {
                   ROS_INFO("%s: Entered command mode",sensor_host.c_str());
               }
               CmdGetParameterSet pget(0,pkt_num++);
               res = send_and_wait(pget,4);
               if (res) {
                   assert(response && (response->cmd == LXS_RESP_GET_PARAMETER_SET));
                   std::shared_ptr<RespParameterSet> resp = std::dynamic_pointer_cast<RespParameterSet>(response);
                   CmdSetTaskParameterSet pset(*resp,pkt_num++);
                   if (rate_control>0) {
                       pset.enable_trigger = true;
                   } else {
                       pset.enable_trigger = trigger_input;
                   }
                   pset.enable_cascading = cascade_output;
                   pset.print(sensor_host);
                   pset.sync();
                   res = send_and_wait(pset,4);
               }
               CmdSetInspectionTask sit(0,true,pkt_num++);
               res = send_and_wait(sit,4);
               CmdExitCommandMode exit(pkt_num++);
               res = send_and_wait(exit,4);
               if (res) {
                   ROS_INFO("%s: Exited command mode",sensor_host.c_str());
               }

               if (rate_control>0) {
                   activate_trigger = true;
               }
               ros::spin();
               activate_trigger = false;
               CmdDisconnectFromSensor disconnect(pkt_num++);
               res = send_and_wait(disconnect,4);
               ROS_INFO("Disconnected");
           } catch (std::exception& e) {
               ROS_ERROR("%s: Exception %s",sensor_host.c_str(),e.what());
           }

           delete receiveThread;
           receiveThread = NULL;
           send_socket.reset();
           receiver_endpoint.reset();
       }

};


boost::asio::io_service LxSDriver::io_service;

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"lxs_driver");
    ros::NodeHandle nh("~");
    LxSDriver driver(nh);

    driver.run();

    return 0;
}
