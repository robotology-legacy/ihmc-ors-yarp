// Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
// Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#ifndef YARP_BRIDGE_IHMC_ORS_H
#define YARP_BRIDGE_IHMC_ORS_H

// YARP includes
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Quaternion.h>

// .idl-generated messages
#include <robotDesired.h>
#include <robotFeedback.h>

// Socket and ports communication
#include <iostream>
#include <asio.hpp>

// FastCDR includes for data serialization
#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

// Standard libraries includes
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>


namespace yarp {

namespace dev {

class BridgeIHMCORS;

class BridgeIHMCORSReceiverThread : public yarp::os::Thread
{
    BridgeIHMCORS * m_parentDevice;

    asio::ip::udp::socket * m_desiredSocket;
    asio::ip::udp::endpoint m_desiredEndpoint;
    asio::io_service * m_desiredService;

    // FastCDR buffer for serialization
    it::iit::yarp::RobotDesireds m_robotDesired;
    eprosima::fastcdr::FastBuffer m_desiredBuffer;


public:
    // Constructor
    BridgeIHMCORSReceiverThread();

    // Configure method
    bool configure(BridgeIHMCORS * parentDevice, yarp::os::Searchable& config);

    // Method to resize the internal buffers
    void resize(const int nrOfJoints);

    // Documented in yarp::os::Thread
    virtual void run();

    // Document in yarp::os::Thread
    virtual void threadRelease();
};

/**
 * \section bridgeIHMCORS
 *
 * A YARP device that exposes a robot composed by a set of YARP devices to a IHMC-ORS based controller.
 *
 *
 *  The parameters taken in input by this device are:
 * | Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
 * |:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
 * | period         |      -         | double            |   s   | 0.005         | No       | Period at which the feedback collected by the robot devices is sent to the  IHMC-ORS controller. | |
 * | desired-timeout  |      -         | double            |   s   | 0.100         | No       | If no desired message has been received for this time, switch back the control mode of the joints to position control.  | |
 * | feedback-address |      -         | string            |   -   |   -           | Yes      | IP address to which to send the feedback datagram. | |
 * | feedback-port-number |  -         | string            |   -   |   9970           | No      | Port number to which to send the feedback datagram. | |
 * | desired-address |  -          | string                |   -   |   -           | Yes      | IP address (of the current machine) on which to listen for incoming desired joints datagrams. | |
 * | desired-port-number |  -          | string            |   -   |   9980           | No      | Port number on which to listen for incoming desired joints datagrams. | |
 */
class BridgeIHMCORS :  public yarp::dev::DeviceDriver,
                       public yarp::dev::IMultipleWrapper,
                       public yarp::os::RateThread
{
private:
    // Devices used to interface with motor controlboard devices
    struct
    {
        yarp::dev::IEncoders             * encs;
        yarp::dev::ITorqueControl        * trqs;
        yarp::dev::IAxisInfo             * axis;
        yarp::dev::IControlMode2         * ctrlMode;
        yarp::dev::IPositionDirect       * posdir;

    } m_wholeBodyControlBoardInterfaces;

    class PortReader;
    // Ports to read IMU(s)
    std::vector<std::shared_ptr<PortReader>> m_imuPorts;
    // Ports to read FTs
    std::vector<std::shared_ptr<PortReader>> m_ftPorts;

    // Helper method to setup port
    bool configurePortBasedMeasurements(const std::string& parameter,
                                        const yarp::os::Searchable& config,
                                        std::vector<std::vector<double>> &outputBuffer,
                                        std::vector<std::shared_ptr<PortReader>>& configuredPorts);

    // Helper methods to deal with C++ interfaces
    bool attachWholeBodyControlBoard(const PolyDriverList& p);
    void resetInterfaces();

    yarp::math::Quaternion quaternionFromRPY(const yarp::sig::Vector& rpyBuffer) const;

    std::vector<yarp::dev::JointTypeEnum> m_jointTypes;

    // Local buffers for readings sensors, populated in run or port reading callbacks
    std::mutex m_sensorReadingsMutex;
    std::atomic<bool> m_sensorsReadingsAvailable;
    std::vector<double> m_jointPositionsFromYARPInDeg;
    std::vector<double> m_jointVelocitiesFromYARPInDegPerSec;
    std::vector<double> m_jointTorquesFromYARP;
    std::vector<std::vector<double>> m_imusReadings;
    std::vector<std::vector<double>> m_ftsReadings;
    std::vector<int> m_desiredControlModesJntForTimeout;
    std::vector<int> m_desiredControlModesForTimeout;

    // Local buffers for sending desired values, populated in onDesiredMessageReceived
    std::vector<int> m_measuredControlModes;
    std::vector<int> m_controlModeTorque;
    std::vector<int> m_controlModePosition;
    std::vector<int> m_desiredControlModesJnt;
    std::vector<int> m_desiredControlModes;
    std::vector<int> m_desTorquesJnt;
    std::vector<double> m_desTorques;
    std::vector<int> m_desPosJnt;
    std::vector<double> m_desPos;

    mutable yarp::sig::Vector m_rpyBuffer;
    mutable yarp::sig::Matrix m_orientationBuffer;

    // FastRTPS robot feedback message
    it::iit::yarp::RobotFeedback m_robotFeedback;
    eprosima::fastcdr::FastBuffer m_feedbackBuffer;

    // Variables for i/o communication
    asio::io_service * m_feedbackService;

    // Sockets and endpoints
    asio::ip::udp::socket * m_feedbackSocket;
    asio::ip::udp::endpoint m_feedbackEndpoint;

    // Thread used for listening on incoming desired joint messages
    BridgeIHMCORSReceiverThread m_receivingThread;

    // Variables to handle the timeout of received messages
    double m_desiredTimeoutInSeconds;
    std::vector< it::iit::yarp::ORSControlMode > m_jointORSControlMode;
    std::mutex m_jointORSControlModeMutex;
    std::atomic<double> m_lastTimeOfReceivedDesiredMessage;

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

    // Function to call when a new desired message has been received
    void onDesiredMessageReceived(const it::iit::yarp::RobotDesireds& receivedRobotDesired);
};

}
}

#endif
