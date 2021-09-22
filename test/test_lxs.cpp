
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <set>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "LxS_Lib/LxS_Datagram.h"

using boost::asio::ip::udp;
using namespace LxS;

boost::asio::io_service io_service;

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
    printf("P");fflush(stdout);
}

void publishPointCloud(const RespZXCoordinates & dxz) {
    printf("Q");fflush(stdout);
}

void * recv_thread(void * dummy) {
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
                    printf("\nRecv: ");
                    dg.dump();
                    dump_buf(recv_buf.begin(),recv_buf.begin()+len);
                    handleCmdDatagram(dg);
                    
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


int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: client <host>" << std::endl;
      return 1;
    }


    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), argv[1], "9008");
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    udp::socket socket(io_service);
    socket.open(udp::v4());



    pthread_t tid;
    pthread_create(&tid,NULL,recv_thread,NULL);

    unsigned int pkt_num = 1;

    while (1) {
        bool res = true;
        sleep(1);
        printf("\nSend: ");
        if (pkt_num & 1) {
            CmdConnectToSensor connect(pkt_num);
            res = send_and_wait(connect,socket,receiver_endpoint);
        } else {
            CmdDisconnectFromSensor disconnect(pkt_num);
            res = send_and_wait(disconnect,socket,receiver_endpoint);
        }
        pkt_num += 1;
        printf("Command result: %s\n",res?"Success":"Failure");
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
