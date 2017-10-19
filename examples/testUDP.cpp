//
// A simple exapmle on how to use udp connection for sending and receiving data
//
#include <iostream>
#include <asio.hpp>

using asio::ip::udp;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: client <host>" << std::endl;
      return 1;
    }

    // variable declaration
    asio::io_service my_io_service;
    udp::resolver my_resolver(my_io_service);  
    udp::resolver::query my_query(udp::v4(), argv[1], "9780");   
    udp::endpoint receiver_endpoint;
    udp::resolver::iterator iter = my_resolver.resolve(my_query);
    udp::resolver::iterator end;         
    udp::socket socket(my_io_service);
    
    // verify port address and number
    while (iter != end)
    {
    receiver_endpoint = *iter++;
    std::cout << receiver_endpoint << std::endl;
    }
    
    // open socket
    socket.open(udp::v4());

    // simple buffer
    char send_buf[128]; 
    char recv_buf[128]; 
    
    send_buf[0] = 'a';
    recv_buf[0] = 'b';
    
    // send data
    std::cout << "sending data..." << std::endl;
    socket.send_to(asio::buffer(send_buf), receiver_endpoint);

    // setting the sender endpoint address and port number
    udp::endpoint sender_endpoint;
    
    asio::ip::address receiver_address = receiver_endpoint.address();
    sender_endpoint.address(receiver_address);
    sender_endpoint.port(9781);
    
    std::cout << "receiving data..." << std::endl;
    size_t len = socket.receive_from(asio::buffer(recv_buf), sender_endpoint);
    
    std::cout << "done!" << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}