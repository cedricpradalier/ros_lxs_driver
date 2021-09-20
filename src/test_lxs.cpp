
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "LxS_Lib/LxS_Datagram.h"

using boost::asio::ip::udp;
using namespace LxS;

template <class iterator> 
void dump_buf(iterator begin, const iterator & end) {
    while (begin != end) {
        printf("%02X ",*begin);
    }
    printf("\n");
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

    boost::asio::io_service io_service;

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), argv[1], "3113");
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    udp::socket socket(io_service);
    socket.open(udp::v4());


    CmdConnectToSensor d_connect(1);
    std::vector<uint8_t> send_buf(d_connect.serialized_length());
    d_connect.serialize(send_buf.begin(),send_buf.end());
    d_connect.dump();
    printf("Send: ");
    dump_buf(send_buf.begin(),send_buf.end());

    socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

    boost::array<char, 1024> recv_buf;
    udp::endpoint sender_endpoint;
    size_t len = socket.receive_from(
        boost::asio::buffer(recv_buf), sender_endpoint);

    Datagram dg;
    dg.deserialize(recv_buf.begin(),recv_buf.end());

    dg.dump();
    printf("Recv: ");
    dump_buf(recv_buf.begin(),recv_buf.begin()+len);
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
