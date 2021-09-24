
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
        std::string sensor_host;
        std::string sensor_port;
        int host_port;
        std::string base_frame;
        double wait_timeout;
        bool trigger_input;
        bool cascade_output;
        static boost::asio::io_service io_service;

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
            // printf("P");fflush(stdout);
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
            // printf("Q");fflush(stdout);
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
                            printf("\n%s Recv: ",sensor_host.c_str());
                            dg.dump();
                            dump_buf(recv_buf.begin(),recv_buf.begin()+len,"");
                            handleCmdDatagram(dg);

                            break;
                    }
                } else {
                    usleep(1000);
                }
            }
        }

        template <class datagram>
            int send(const datagram & dg,udp::socket & socket, udp::endpoint & ep) {
                std::vector<uint8_t> send_buf(dg.serialized_length());
                dg.serialize(send_buf.begin(),send_buf.end());
#if 1
                dg.dump();
                dump_buf(send_buf.begin(),send_buf.end(),sensor_host);
#endif

                return socket.send_to(boost::asio::buffer(send_buf), ep);
            }

        template <class datagram>
            bool send_and_wait(const datagram & dg,udp::socket & socket, udp::endpoint & ep) {
                std::vector<uint8_t> send_buf(dg.serialized_length());
                dg.serialize(send_buf.begin(),send_buf.end());
#if 1
                dg.dump();
                dump_buf(send_buf.begin(),send_buf.end(),sensor_host);
#endif
                eCmd resp_cmd = eCmdResponse(dg.cmd);

                std::unique_lock<std::mutex> lk(receiveMtx);
                waitingList.insert(LxS::LXS_RESP_ACK_FAILURE);
                waitingList.insert(resp_cmd);
                socket.send_to(boost::asio::buffer(send_buf), ep);
                std::chrono::duration<double> timeout(wait_timeout);
                std::cv_status wstatus = receiveCond.wait_for(lk, timeout);
                bool res = false;
                if (wstatus == std::cv_status::timeout) {
                    ROS_ERROR("%s: Timeout while waiting for answer to %04X (%s)",sensor_host.c_str(),uint16_t(dg.cmd),eCmdString(dg.cmd));
                    res = false;
                } else if (response) {
                    switch (response->cmd) {
                        case LXS_RESP_ACK_SUCCESS:
                            res = true;
                            break;
                        case LXS_RESP_ACK_FAILURE:
                            ROS_ERROR("%s: Failure as answer to %04X (%s): %04X",sensor_host.c_str(),uint16_t(dg.cmd),eCmdString(dg.cmd),dg.status);
                            res = false;
                            break;
                        default:
                            res = true;
                            break;
                    }
                }
                return res;
            }

    public: 
       LxSDriver(ros::NodeHandle & nh) {
           connected = false;
            nh.param("base_frame",base_frame,std::string("lrs36"));
            nh.param("sensor_host",sensor_host,std::string("192.168.60.3"));
            nh.param("sensor_port",sensor_port,std::string("9008"));
            nh.param("host_port",host_port,5634);
            nh.param("wait_timeout",wait_timeout,0.5);
            nh.param("trigger_input",trigger_input,false);
            nh.param("cascade_output",cascade_output,false);
            scan_pub = nh.advertise<sensor_msgs::PointCloud2>("scans",1);
       }

       void run() {
           udp::resolver resolver(io_service);
           udp::resolver::query query(udp::v4(), sensor_host,sensor_port);
           udp::endpoint receiver_endpoint = *resolver.resolve(query);

           udp::socket socket(io_service);
           socket.open(udp::v4());

           receiveThread = new std::thread(&LxSDriver::recv_thread,this);

           unsigned int pkt_num = 1;
           bool res = false;

           connected = false;

           try {
               while (!connected && ros::ok()) {
                   ROS_INFO("Connecting to %s:%s",sensor_host.c_str(),sensor_port.c_str());
                   CmdConnectToSensor connect(pkt_num++);
                   res = send_and_wait(connect,socket,receiver_endpoint);
                   if (res) {
                       connected = true;
                       ROS_INFO("Connected to %s",sensor_host.c_str());
                       break;
                   }
                   CmdDisconnectFromSensor disconnect(pkt_num++);
                   res = send_and_wait(disconnect,socket,receiver_endpoint);
                   ros::Duration(1).sleep();
               }
               CmdEnterCommandMode enter(pkt_num++);
               res = send_and_wait(enter,socket,receiver_endpoint);
               if (res) {
                   ROS_INFO("%s: Entered command mode",sensor_host.c_str());
               }
               CmdGetSingleTaskParameter getTrigger(0xBBBA,pkt_num++);
               res = send_and_wait(getTrigger,socket,receiver_endpoint);
               if (res) {
                   assert(response && (response->cmd == LXS_RESP_TASK_PARAMETER));
                   std::shared_ptr<RespTaskParameter> resp = std::dynamic_pointer_cast<RespTaskParameter>(response);
                   if (bool(resp->getParam().data[0]) != trigger_input) {
                       CmdSetSingleTaskParameter setTrigger(0xBBBA,trigger_input?1:0,true,pkt_num++);
                       res = send_and_wait(setTrigger,socket,receiver_endpoint);
                       if (res) {
                           ROS_INFO("%s: Set triggered mode: %d",sensor_host.c_str(),int(trigger_input));
                       }
                   } else {
                       ROS_INFO("%s: triggered mode already set: %d",sensor_host.c_str(),int(trigger_input));
                   }
               }
               CmdGetSingleTaskParameter getCascade(0xBBBC,pkt_num++);
               res = send_and_wait(getCascade,socket,receiver_endpoint);
               if (res) {
                   assert(response && (response->cmd == LXS_RESP_TASK_PARAMETER));
                   std::shared_ptr<RespTaskParameter> resp = std::dynamic_pointer_cast<RespTaskParameter>(response);
                   if (bool(resp->getParam().data[0]) != trigger_input) {
                       CmdSetSingleTaskParameter setCascade(0xBBBC,cascade_output?1:0,true,pkt_num++);
                       res = send_and_wait(setCascade,socket,receiver_endpoint);
                       if (res) {
                           ROS_INFO("%s: Set cascade output: %d",sensor_host.c_str(),int(cascade_output));
                       }
                   } else {
                       ROS_INFO("%s: cascade mode already set: %d",sensor_host.c_str(),int(trigger_input));
                   }
               }
               CmdExitCommandMode exit(pkt_num++);
               res = send_and_wait(exit,socket,receiver_endpoint);
               if (res) {
                   ROS_INFO("%s: Exited command mode",sensor_host.c_str());
               }

               ros::spin();
               CmdDisconnectFromSensor disconnect(pkt_num++);
               res = send_and_wait(disconnect,socket,receiver_endpoint);
               ROS_INFO("Disconnected");
           } catch (std::exception& e) {
               ROS_ERROR("%s: Exception %s",sensor_host.c_str(),e.what());
           }

           delete receiveThread;
           receiveThread = NULL;
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
