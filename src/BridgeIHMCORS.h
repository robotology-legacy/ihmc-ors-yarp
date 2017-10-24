// Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
// Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#ifndef YARP_BRIDGE_IHMC_ORS_H
#define YARP_BRIDGE_IHMC_ORS_H

// YARP includes
#include <yarp/os/Mutex.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

// .idl-generated messages
#include <robotDesired.h>
#include <robotFeedback.h>

// Socket and ports communication
#include <iostream>
#include <asio.hpp>

// FastCDR includes for data serialization
#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

namespace yarp {
namespace dev {

/**
 * \section bridgeIHMCORS
 *
 * A YARP device that exposes a robot composed by a set of YARP devices to a IHMC-ORS based controller.
 *
 *
 *  The parameters taken in input by this device are:
 * | Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
 * |:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
 * | period         |      -         | double            |   s   | 0.005         | No       | Period at which the feedback collected by the robot devices is sent to the  IHMC-ORS controller | |
 * | remote-address |      -         | string            |   -   |   -           | Yes      | IP address | |
 * | remote-port-number |  -         | string            |   -   |   -           | Yes      | Port number | |
 */
class BridgeIHMCORS :  public yarp::dev::DeviceDriver,
                       public yarp::dev::IMultipleWrapper,
                       public yarp::os::RateThread
{
private:
    //
    yarp::os::Mutex m_deviceMutex;
    // Devices used to interface with motor controlboard devices
    struct
    {
        yarp::dev::IEncoders             * encs;
        yarp::dev::ITorqueControl        * trqs;
        yarp::dev::IAxisInfo             * axis;
        yarp::dev::IControlMode2         * ctrlMode;
    } m_wholeBodyControlBoardInterfaces;

    bool m_correctlyConfigured;

    // Helper methods to deal with C++ interfaces
    bool attachWholeBodyControlBoard(const PolyDriverList& p);
    void resetInterfaces();

    // Local buffers for readings sensors
    std::vector<yarp::dev::JointTypeEnum> m_jointTypes;
    std::vector<double> m_jointPositionsFromYARP;
    std::vector<double> m_jointVelocitiesFromYARP;
    std::vector<double> m_jointTorquesFromYARP;

    // FastRTPS robot feedback message
    it::iit::yarp::RobotFeedback m_robotFeedback;
    it::iit::yarp::RobotDesireds m_robotDesired;

    // Function to call when a new desired message has been received
    // TODO(traversaro): make sure that this function is called
    void onDesiredMessageReceived(const it::iit::yarp::RobotDesireds& receivedRobotDesired);

    // Local buffers for sending desired values
    std::vector<double> m_desiredTorques;

    // Variables for i/o communication
    asio::io_service * m_feedbackService;
    asio::io_service * m_desiredService;

    // Creating sockets and endpoints
    asio::ip::udp::socket * m_feedbackSocket;
    asio::ip::udp::socket * m_desiredSocket;
    asio::ip::udp::endpoint m_feedbackEndpoint;
    asio::ip::udp::endpoint m_desiredEndpoint;

    // FastCDR buffer for serialization
    eprosima::fastcdr::FastBuffer m_feedbackBuffer;
    eprosima::fastcdr::FastBuffer m_desiredBuffer;

public:
    // CONSTRUCTOR
    BridgeIHMCORS();
    ~BridgeIHMCORS();

    // DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IMULTIPLE WRAPPER
    virtual bool attachAll(const yarp::dev::PolyDriverList &p);
    virtual bool detachAll();

    // RATE THREAD
    virtual void run();

};

}
}

#endif
