// Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
// Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#include <cmath>
#include <string>

#include <rtf/Asserter.h>
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>

#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/rtf/TestCase.h>

#include <iostream>
#include <asio.hpp>

// FastCDR includes for data serialization
#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include <robotDesired.h>
#include <robotFeedback.h>


using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using asio::ip::udp;


/**********************************************************************/
class BridgeIHMCORSTest : public yarp::rtf::TestCase
{
    // Driver interface to the robot
    yarp::dev::PolyDriver       m_robot;
    yarp::dev::IEncoders       *m_iencs;
    yarp::dev::ITorqueControl  *m_itrq;
    yarp::dev::IAxisInfo       *m_iaxis;
    yarp::dev::IPositionDirect *m_iposdir;
    yarp::dev::IControlMode2   *m_ctrlmode;

    int nj;

public:
    /******************************************************************/
    BridgeIHMCORSTest() :
            TestCase("BridgeIHMCORSTest")
    {
    }

    /******************************************************************/
    virtual ~BridgeIHMCORSTest()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        yarp::os::Property deviceOptions;
        deviceOptions.put("device","remotecontrolboardremapper");
        Bottle axesNames;
        Bottle & axesList = axesNames.addList();
        axesList.addString("torso_pitch");
        axesList.addString("torso_roll");
        axesList.addString("torso_yaw");
        axesList.addString("l_shoulder_pitch");
        axesList.addString("l_shoulder_roll");
        axesList.addString("l_shoulder_yaw");
        axesList.addString("l_elbow");
        axesList.addString("r_shoulder_pitch");
        axesList.addString("r_shoulder_roll");
        axesList.addString("r_shoulder_yaw");
        axesList.addString("r_elbow");
        axesList.addString("l_hip_pitch");
        axesList.addString("l_hip_roll");
        axesList.addString("l_hip_yaw");
        axesList.addString("l_knee");
        axesList.addString("l_ankle_pitch");
        axesList.addString("l_ankle_roll");
        axesList.addString("r_hip_pitch");
        axesList.addString("r_hip_roll");
        axesList.addString("r_hip_yaw");
        axesList.addString("r_knee");
        axesList.addString("r_ankle_pitch");
        axesList.addString("r_ankle_roll");

        deviceOptions.put("axesNames", axesNames.get(0));
        Bottle remoteControlBoards;
        Bottle & remoteControlBoardsList = remoteControlBoards.addList();
        remoteControlBoardsList.addString("/icubSim/torso");
        remoteControlBoardsList.addString("/icubSim/head");
        remoteControlBoardsList.addString("/icubSim/left_arm");
        remoteControlBoardsList.addString("/icubSim/right_arm");
        remoteControlBoardsList.addString("/icubSim/left_leg");
        remoteControlBoardsList.addString("/icubSim/right_leg");

        deviceOptions.put("remoteControlBoards",remoteControlBoards.get(0));
        deviceOptions.put("localPortPrefix","/BridgeIHMCORSTest");

        Property & remoteControlBoardsOpts = deviceOptions.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteControlBoardsOpts.put("writeStrict","on");

        // For the test we hardcode the considered joints
        RTF_ASSERT_ERROR_IF_FALSE(m_robot.open(deviceOptions), Asserter::format("Unable to open robot remote_controlboard"));
        RTF_ASSERT_ERROR_IF_FALSE(m_robot.view(m_iencs),  Asserter::format("Unable to view IEncoder interface"));
        RTF_ASSERT_ERROR_IF_FALSE(m_robot.view(m_itrq),  Asserter::format("Unable to view ITorqueControl interface"));
        RTF_ASSERT_ERROR_IF_FALSE(m_robot.view(m_iaxis),  Asserter::format("Unable to view IAxisInfo interface"));
        RTF_ASSERT_ERROR_IF_FALSE(m_robot.view(m_iposdir),  Asserter::format("Unable to view IPositionDirect interface"));
        RTF_ASSERT_ERROR_IF_FALSE(m_robot.view(m_ctrlmode),  Asserter::format("Unable to view IControlMode2 interface"));
        RTF_ASSERT_ERROR_IF_FALSE(m_iencs->getAxes(&nj),  Asserter::format("getAxes failed"));

        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        m_robot.close();
    }

    /******************************************************************/
    virtual void run()
    {
        RTF_TEST_REPORT("Setup UDP interface");

        // Variables declaration
        asio::io_service sender_io_service;
        asio::io_service receiver_io_service;

        udp::resolver sender_resolver(sender_io_service);
        udp::resolver receiver_resolver(receiver_io_service);

        udp::resolver::query sender_query(udp::v4(), "localhost", "9980");
        udp::resolver::query receiver_query(udp::v4(), "localhost", "9970");

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
        it::iit::yarp::RobotFeedback receiver_data;
        it::iit::yarp::RobotDesireds sender_data;

        // Resize buffers
        sender_data.jointDesireds().resize(nj);

        // Receiver
        receiver_buffer.resize(receiver_data.getMaxCdrSerializedSize());

        //////////////
        // Switch to TORQUE_CONTROL mode (VOCAB_CM_TORQUE)
        //////////////

        // Filling the vector of data to be sent to the robot
        for (size_t jnt=0; jnt < nj; jnt++)
        {
            sender_data.jointDesireds()[jnt].controlMode() = it::iit::yarp::TORQUE_CONTROL;
        }

        // Send the desired message (setting the message torque)
        eprosima::fastcdr::FastBuffer sender_buffer;
        eprosima::fastcdr::Cdr sender_data_ser(sender_buffer);
        sender_data.serialize(sender_data_ser);

        RTF_TEST_REPORT("Send message switching to TORQUE_CONTROL");
        sender_socket.send_to(asio::buffer(sender_buffer.getBuffer(), sender_buffer.getBufferSize()), sender_endpoint);

        yarp::os::Time::delay(1.0);

        // Check that the controlmodes have switched
        std::vector<int> measuredControlModes(nj);

        m_ctrlmode->getControlModes(measuredControlModes.data());

        for (size_t jnt=0; jnt < nj; jnt++)
        {
            RTF_ASSERT_ERROR_IF_FALSE(measuredControlModes[jnt] == VOCAB_CM_TORQUE, Asserter::format("Unable to swich joint %d to VOCAB_CM_TORQUE", jnt));
        }

        //////////////
        // Switch to POSITION_DIRECT mode (VOCAB_CM_POSITION_DIRECT)
        //////////////

        // Filling the vector of data to be sent to the robot
        for (size_t jnt=0; jnt < nj; jnt++)
        {
            sender_data.jointDesireds()[jnt].controlMode() = it::iit::yarp::POSITION_CONTROL;
        }

        // Send the desired message
        // TODO(traversaro) : cleanup all this buffers
        eprosima::fastcdr::FastBuffer sender_buffer2;
        eprosima::fastcdr::Cdr sender_data_ser2(sender_buffer2);
        sender_data.serialize(sender_data_ser2);

        RTF_TEST_REPORT("Send message switching to POSITION_CONTROL");
        sender_socket.send_to(asio::buffer(sender_buffer2.getBuffer(), sender_buffer2.getBufferSize()), sender_endpoint);

        yarp::os::Time::delay(1.0);

        // Check that the YARP controlmodes have switched
        m_ctrlmode->getControlModes(measuredControlModes.data());

        for (size_t jnt=0; jnt < nj; jnt++)
        {
            RTF_ASSERT_ERROR_IF_FALSE(measuredControlModes[jnt] == VOCAB_CM_POSITION_DIRECT, Asserter::format("Unable to swich joint %d to VOCAB_CM_POSITION_DIRECT", jnt));
        }

        //////////////
        // Switch to NOT_ENABLED mode (VOCAB_CM_POSITION)
        //////////////

        // Filling the vector of data to be sent to the robot
        for (size_t jnt=0; jnt < nj; jnt++)
        {
            sender_data.jointDesireds()[jnt].controlMode() = it::iit::yarp::NOT_ENABLED;
        }

        // Send the desired message
        RTF_TEST_REPORT("Send message switching to NOT_ENABLED");
        // TODO(traversaro) : cleanup all this buffers
        eprosima::fastcdr::FastBuffer sender_buffer3;
        eprosima::fastcdr::Cdr sender_data_ser3(sender_buffer3);
        sender_data.serialize(sender_data_ser3);
        sender_socket.send_to(asio::buffer(sender_buffer3.getBuffer(), sender_buffer3.getBufferSize()), sender_endpoint);

        yarp::os::Time::delay(1.0);

        // Check that the YARP controlmodes have switched
        m_ctrlmode->getControlModes(measuredControlModes.data());

        for (size_t jnt=0; jnt < nj; jnt++)
        {
            RTF_ASSERT_ERROR_IF_FALSE(measuredControlModes[jnt] == VOCAB_CM_POSITION, Asserter::format("Unable to switch joint %d to VOCAB_CM_POSITION", jnt));
        }

    }
};

PREPARE_PLUGIN(BridgeIHMCORSTest)
