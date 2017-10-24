// A simple test for verifying both sender and receiver part are correctly
// implemented inside BridgeIHCORS.cpp.

// Asio includes
#include <iostream>
#include <asio.hpp>

// FastCDR includes for data serialization
#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

// .idl-generated messages
#include <robotDesired.h>
#include <robotFeedback.h>

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

    // Variables declaration
    asio::io_service sender_io_service;
    asio::io_service receiver_io_service;

    udp::resolver sender_resolver(sender_io_service);
    udp::resolver receiver_resolver(receiver_io_service);

    udp::resolver::query sender_query(udp::v4(), argv[1], "9888");
    udp::resolver::query receiver_query(udp::v4(), argv[1], "9876");

    udp::endpoint sender_endpoint;
    udp::endpoint receiver_endpoint;

    // Trying to match the specified IP address and port number to an endpoint
    sender_endpoint = *sender_resolver.resolve(sender_query);
    receiver_endpoint = *receiver_resolver.resolve(receiver_query);

    // Opening sockets
    udp::socket sender_socket(sender_io_service);
    udp::socket receiver_socket(receiver_io_service);

    sender_socket.open(udp::v4());
    receiver_socket.open(udp::v4());

    receiver_socket.bind(receiver_endpoint);

    // FastCDR buffers and serialization
    eprosima::fastcdr::FastBuffer receiver_buffer;
    eprosima::fastcdr::FastBuffer sender_buffer;
    it::iit::yarp::RobotFeedback receiver_data;
    it::iit::yarp::RobotDesireds sender_data;

    // Define size of data to be sent
    int nj=23;

    // Resize buffers
    sender_data.jointDesireds().resize(nj);

    // Receiver
    receiver_buffer.resize(receiver_data.getMaxCdrSerializedSize());

    //Filling the vector of data to be sent to the robot
    for (size_t jnt=0; jnt < nj; jnt++)
    {
        sender_data.jointDesireds()[jnt].tau() = 1;
    }

    while(1)
    {
        // Serialize sender_data and send message using udp
        eprosima::fastcdr::Cdr sender_data_ser(sender_buffer);
        sender_data.serialize(sender_data_ser);
        sender_socket.send_to(asio::buffer(sender_buffer.getBuffer(), sender_buffer.getBufferSize()), sender_endpoint);
        std::cout << "Sending data... " <<  sender_data.jointDesireds()[2].tau() << std::endl;

        // Receive data
        size_t len = receiver_socket.receive_from(asio::buffer(receiver_buffer.getBuffer(), receiver_buffer.getBufferSize()), receiver_endpoint);
        eprosima::fastcdr::Cdr receiver_data_ser(receiver_buffer);
        receiver_data.deserialize(receiver_data_ser);
        std::cout << "Receving data... " <<  receiver_data.jointStates()[2].q() << std::endl;
    }

  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
