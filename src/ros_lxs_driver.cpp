
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <set>
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
        std::string host_name;
        std::string base_frame;
        static boost::asio::io_service io_service;

        template <class iterator> 
            void dump_buf(iterator begin, const iterator & end) {
                while (begin != end) {
                    printf("%02X ",*begin);
                    begin ++;
                }
                printf("\n");
            }

        pthread_mutex_t waitingListMtx = PTHREAD_MUTEX_INITIALIZER;
        pthread_cond_t waitingListCond = PTHREAD_COND_INITIALIZER;
        std::set<eCmd> waitingList;
        LxS::DatagramPtr response;

        template <class datagram>
            void handleCmdDatagram(const datagram & dg) {
                pthread_mutex_lock(&waitingListMtx);
                std::set<eCmd>::iterator it = waitingList.find(dg.cmd);
                if (it != waitingList.end()) {
                    response = convertToInheritedDatagram(dg);
                    waitingList.clear();
                    pthread_cond_signal(&waitingListCond);
                }
                pthread_mutex_unlock(&waitingListMtx);
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

        static void * recv_thread(void * dummy) {
            LxSDriver *that = (LxSDriver*)dummy;
            udp::socket socket(io_service, udp::endpoint(udp::v4(), 5634));
            uint8_t axis = 0;
            RespDataX dx;
            RespDataZ dz;
            RespZXCoordinates dxz;
            while (true) {
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
                                    that->publishPointCloud(dx,dz);
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
                                    that->publishPointCloud(dx,dz);
                                    axis = 0;
                                }
                            }
                            break;
                        case LXS_RESP_XZ_HIGH_DATA:
                            {
                                // printf("I");fflush(stdout);
                                dxz = RespZXCoordinates(dg);
                                that->publishPointCloud(dxz);
                            }
                            break;
                        default:
                            printf("\nRecv: ");
                            dg.dump();
                            that->dump_buf(recv_buf.begin(),recv_buf.begin()+len);
                            that->handleCmdDatagram(dg);

                            break;
                    }
                } else {
                    usleep(1000);
                }
            }
            return NULL;
        }

        template <class datagram>
            int send(const datagram & dg,udp::socket & socket, udp::endpoint & ep) {
                std::vector<uint8_t> send_buf(dg.serialized_length());
                dg.serialize(send_buf.begin(),send_buf.end());
#if 1
                dg.dump();
                dump_buf(send_buf.begin(),send_buf.end());
#endif

                return socket.send_to(boost::asio::buffer(send_buf), ep);
            }

        template <class datagram>
            bool send_and_wait(const datagram & dg,udp::socket & socket, udp::endpoint & ep) {
                std::vector<uint8_t> send_buf(dg.serialized_length());
                dg.serialize(send_buf.begin(),send_buf.end());
#if 1
                dg.dump();
                dump_buf(send_buf.begin(),send_buf.end());
#endif
                eCmd resp_cmd = eCmdResponse(dg.cmd);

                pthread_mutex_lock(&waitingListMtx);
                waitingList.insert(LxS::LXS_RESP_ACK_FAILURE);
                waitingList.insert(resp_cmd);
                socket.send_to(boost::asio::buffer(send_buf), ep);
                pthread_cond_wait(&waitingListCond,&waitingListMtx);
                bool res = false;
                if (response) {
                    switch (response->cmd) {
                        case LXS_RESP_ACK_SUCCESS:
                            res = true;
                            break;
                        case LXS_RESP_ACK_FAILURE:
                            res = false;
                            break;
                        default:
                            res = true;
                            break;
                    }
                }
                response.reset();
                pthread_mutex_unlock(&waitingListMtx);
                return res;
            }

    public: 
       LxSDriver(ros::NodeHandle & nh) {
           connected = false;
            nh.param("base_frame",base_frame,std::string("lpr36"));
            nh.param("host_name",host_name,std::string("192.168.60.3"));
            scan_pub = nh.advertise<sensor_msgs::PointCloud2>("scans",1);
       }

       void run() {
           udp::resolver resolver(io_service);
           udp::resolver::query query(udp::v4(), host_name, "9008");
           udp::endpoint receiver_endpoint = *resolver.resolve(query);

           udp::socket socket(io_service);
           socket.open(udp::v4());

           pthread_t tid;
           pthread_create(&tid,NULL,LxSDriver::recv_thread,this);

           unsigned int pkt_num = 1;
           bool res = false;

           connected = false;

           try {
               while (!connected && ros::ok()) {
                   ROS_INFO("Connecting to %s",host_name.c_str());
                   CmdConnectToSensor connect(pkt_num++);
                   res = send_and_wait(connect,socket,receiver_endpoint);
                   if (res) {
                       connected = true;
                       ROS_INFO("Connected");
                       break;
                   }
                   CmdDisconnectFromSensor disconnect(pkt_num++);
                   res = send_and_wait(disconnect,socket,receiver_endpoint);
                   ros::Duration(1).sleep();
               }
               ros::spin();
               CmdDisconnectFromSensor disconnect(pkt_num++);
               res = send_and_wait(disconnect,socket,receiver_endpoint);
               ROS_INFO("Disconnected");
           } catch (std::exception& e) {
               ROS_ERROR("Exception %s",e.what());
           }
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
