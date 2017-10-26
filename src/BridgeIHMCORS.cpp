// Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
// Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#include "BridgeIHMCORS.h"

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

#include <cassert>
#include <climits>
#include <cmath>

namespace yarp
{
namespace dev
{

BridgeIHMCORSReceiverThread::BridgeIHMCORSReceiverThread(): m_parentDevice(nullptr),
                                                            m_desiredSocket(nullptr),
                                                            m_desiredService(nullptr)
{}

bool BridgeIHMCORSReceiverThread::configure(BridgeIHMCORS *parentDevice, yarp::os::Searchable& config)
{
    m_parentDevice = parentDevice;

    // Configure inbound socket (receiving commands)
    yarp::os::Value desiredPortNumber = config.check("desired-port-number", yarp::os::Value("9980"));
    yarp::os::Value desiredAddressVal = config.find("desired-address");

    if (desiredAddressVal.isNull())
    {
        yError() << "IP address for receiving data not found";
        return false;
    }

    // Trying to match the specified IP address and port number to an endpoint
    m_desiredService = new asio::io_service;
    asio::ip::udp::resolver resolverDesired(*m_desiredService);
    asio::ip::udp::resolver::query queryDesired(asio::ip::udp::v4(), desiredAddressVal.asString(), desiredPortNumber.asString());
    m_desiredEndpoint = *resolverDesired.resolve(queryDesired);

    // TODO(nava): substitute this with an error message in case the parameter is not found
    std::cout << "Current endpoint for receiving data:" << m_desiredEndpoint << std::endl;

    m_desiredSocket = new asio::ip::udp::socket(*m_desiredService);

    if (!m_desiredSocket)
    {
        yError() << "Fail to create the ""desired"" socket";
        return false;
    }
    m_desiredSocket->open(asio::ip::udp::v4());
    m_desiredSocket->bind(m_desiredEndpoint);

    return true;
}

void BridgeIHMCORSReceiverThread::resize(const int nrOfJoints)
{
    m_robotDesired.jointDesireds().resize(nrOfJoints);
    m_desiredBuffer.resize(m_robotDesired.getMaxCdrSerializedSize());
}

void BridgeIHMCORSReceiverThread::run()
{
    while (!isStopping())
    {
        // Deserialize received data (desired)
        size_t len = m_desiredSocket->receive_from(
                asio::buffer(m_desiredBuffer.getBuffer(), m_desiredBuffer.getBufferSize()), m_desiredEndpoint);
        eprosima::fastcdr::Cdr robotDesiredSer(m_desiredBuffer);
        m_robotDesired.deserialize(robotDesiredSer);

        // Fill m_desiredTorques vector and send references to the robot
        m_parentDevice->onDesiredMessageReceived(m_robotDesired);
    }
}

void BridgeIHMCORSReceiverThread::threadRelease()
{
    m_parentDevice = nullptr;
    delete m_desiredSocket;
    m_desiredSocket = nullptr;
    delete m_desiredService;
    m_desiredService = nullptr;
}

BridgeIHMCORS::BridgeIHMCORS(): os::RateThread(5), m_feedbackSocket(nullptr)
{
    resetInterfaces();
}

BridgeIHMCORS::~BridgeIHMCORS()
{

}

bool BridgeIHMCORS::open(yarp::os::Searchable& config)
{
    using asio::ip::udp;

    // Get period
    double periodInSeconds=config.check("period",os::Value(0.005)).asDouble();
    int period_ms=int(1000.0*periodInSeconds);
    this->setRate(period_ms);

    // Get desired timeout
    m_desiredTimeoutInSeconds=config.check("desired-timeout",os::Value(0.1)).asDouble();

    // Configure sockets

    // Configure outbound socket (sending references to remote)
    yarp::os::Value feedbackPortNumber = config.check("feedback-port-number", yarp::os::Value("9970"));
    yarp::os::Value feedbackAddressVal = config.find("feedback-address");

    if (feedbackPortNumber.isNull())
    {
        yError() << "Port number for sending feedback not found";
        return false;
    }
    if (feedbackAddressVal.isNull())
    {
        yError() << "IP address for sending feedback not found";
        return false;
    }

    // Trying to match the specified IP address and port number to an endpoint
    m_feedbackService = new asio::io_service;
    udp::resolver resolver(*m_feedbackService);
    udp::resolver::query query(udp::v4(), feedbackAddressVal.asString(), feedbackPortNumber.asString());
    m_feedbackEndpoint = *resolver.resolve(query);

    // TODO(nava): substitute this with an error message in case the parameter is not found
    std::cout << "Current endpoint for sending feedback:" << m_feedbackEndpoint << std::endl;

    // Opening feedback socket
    m_feedbackSocket = new udp::socket(*m_feedbackService);

    if (!m_feedbackSocket)
    {
        yError() << "Fail to create the ""feedback"" socket";
        return false;
    }

    m_feedbackSocket->open(udp::v4());

    // Configuring receiving thread
    m_receivingThread.configure(this, config);

    return true;
}

bool BridgeIHMCORS::attachAll(const PolyDriverList& p)
{
    bool ok = true;
    ok = ok && attachWholeBodyControlBoard(p);

    if (ok)
    {
        // If everything is ok, start the threads

        // Thread streaming the feedback
        this->start();

        // Thread receiving the desired values
        m_receivingThread.start();
    }

    return ok;
}

bool BridgeIHMCORS::attachWholeBodyControlBoard(const PolyDriverList& p)
{
    bool foundDevice = false;

    // We assume that only one device in the device list passed is a controlboard,
    // and that it contains all the joint of the robot.
    // If you want to control with the IHMC bridge only a subset of the joints
    // or the union of the joints exposed by two different devices, you can
    // combine them using the controlboardremapper
    for (size_t devIdx = 0; devIdx < (size_t) p.size(); devIdx++)
    {
        if (p[devIdx]->poly->view(m_wholeBodyControlBoardInterfaces.encs))
        {
            // The considered device implements IEncoders, let's see if it implements all the other interfaces we are interested in
            bool ok = p[devIdx]->poly->view(m_wholeBodyControlBoardInterfaces.trqs);
            ok = ok && p[devIdx]->poly->view(m_wholeBodyControlBoardInterfaces.axis);
            ok = ok && p[devIdx]->poly->view(m_wholeBodyControlBoardInterfaces.ctrlMode);

            // Read number of joints to resize buffers
            int nj=0;
            ok = ok && m_wholeBodyControlBoardInterfaces.encs->getAxes(&nj);

            // Read joint types
            if (ok)
            {
                m_jointTypes.resize(nj);
                for(int jnt=0; jnt < nj; jnt++)
                {
                    ok = ok && m_wholeBodyControlBoardInterfaces.axis->getJointType(jnt, m_jointTypes[jnt]);
                }
            }

            m_jointPositionsFromYARPInDeg.resize(nj);
            m_jointVelocitiesFromYARPInDegPerSec.resize(nj);
            m_jointTorquesFromYARP.resize(nj);

            m_measuredControlModes.resize(nj);
            m_controlModeTorque.resize(nj, VOCAB_CM_TORQUE);
            m_controlModePosition.resize(nj, VOCAB_CM_POSITION);
            m_desiredTorques.resize(nj);

            // Resize buffers
            m_robotFeedback.jointStates().resize(nj);
            m_robotFeedback.forceSensors().resize(0);
            m_robotFeedback.imuStates().resize(0);


            // Resize FastCDR feedback buffer to the size of m_robotFeedback
            m_feedbackBuffer.resize(m_robotFeedback.getCdrSerializedSize(m_robotFeedback));


            // Resize the internal buffers of the receiving thread
            m_receivingThread.resize(nj);

            foundDevice = ok;
            break; // see assumption
        }
    }

    if (!foundDevice)
    {
        yError() << "bridgeIHMCORS: impossible to find device that implements IEncoders, ITorqueControl and IAxisInfo interfaces.";
    }

    return foundDevice;
}

double deg2rad(const double angleInDeg)
{
    return angleInDeg*M_PI/180.0;
}

double rad2deg(const double angleInRad)
{
    return angleInRad*180.0/M_PI;
}


void BridgeIHMCORS::run()
{
    using eprosima::fastcdr::Cdr;

    bool sensorsReadCorrectly = true;
    m_sensorReadingsMutex.lock();
    sensorsReadCorrectly = sensorsReadCorrectly && m_wholeBodyControlBoardInterfaces.encs->getEncoders(m_jointPositionsFromYARPInDeg.data());
    sensorsReadCorrectly = sensorsReadCorrectly && m_wholeBodyControlBoardInterfaces.encs->getEncoderSpeeds(m_jointVelocitiesFromYARPInDegPerSec.data());
    sensorsReadCorrectly = sensorsReadCorrectly && m_wholeBodyControlBoardInterfaces.trqs->getTorques(m_jointTorquesFromYARP.data());
    m_sensorReadingsMutex.unlock();

    if (sensorsReadCorrectly)
    {
        m_sensorsReadingsAvailable = true;

        for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
        {
            if (m_jointTypes[jnt] == VOCAB_JOINTTYPE_REVOLUTE)
            {
                m_robotFeedback.jointStates()[jnt].q() = deg2rad(m_jointPositionsFromYARPInDeg[jnt]);
                m_robotFeedback.jointStates()[jnt].qd() = deg2rad(m_jointVelocitiesFromYARPInDegPerSec[jnt]);
            }
            else
            {
                m_robotFeedback.jointStates()[jnt].q() = m_jointPositionsFromYARPInDeg[jnt];
                m_robotFeedback.jointStates()[jnt].qd() = m_jointVelocitiesFromYARPInDegPerSec[jnt];
            }

           m_robotFeedback.jointStates()[jnt].tau() = m_jointTorquesFromYARP[jnt];
           // std::cerr << "m_robotFeedback message updated " << std::endl;
        }

       // Serialize m_robotFeedback and send message using udp
       Cdr robotFeedbackSer(m_feedbackBuffer);
       m_robotFeedback.serialize(robotFeedbackSer);
       m_feedbackSocket->send_to(asio::buffer(m_feedbackBuffer.getBuffer(), m_feedbackBuffer.getBufferSize()), m_feedbackEndpoint);
    }
    else
    {
        yWarning() << "BridgeIHMCORS: some sensor were not read correctly";
    }

    // Handle timeout of desired messages
    if (m_controlActive)
    {
        double timeSinceLastFeedback = yarp::os::Time::now()-m_lastTimeOfReceivedDesiredMessage;

        if (timeSinceLastFeedback > m_desiredTimeoutInSeconds)
        {
            m_controlActive = false;
            m_wholeBodyControlBoardInterfaces.ctrlMode->setControlModes(m_controlModePosition.data());
            yInfo("BridgeIHMCORS: %lf seconds passed without receiving a message from the IHMC-ORS controller, switching back the robot to position mode.", timeSinceLastFeedback);
        }
    }
}

bool BridgeIHMCORS::detachAll()
{
    // Stop the threads

    // Stop streaming thread and wait for termination
    if (isRunning())
    {
        stop();
    }

    // Stop receiving thread and wait for termination
    if (m_receivingThread.isRunning())
    {
        m_receivingThread.stop();
    }

    resetInterfaces();

    return true;
}

bool BridgeIHMCORS::close()
{
    if (m_feedbackSocket)
    {
        m_feedbackSocket->close();
        delete m_feedbackSocket;
        m_feedbackSocket = nullptr;
    }

    return true;
}

void BridgeIHMCORS::resetInterfaces()
{
    m_wholeBodyControlBoardInterfaces.encs = nullptr;
    m_wholeBodyControlBoardInterfaces.trqs = nullptr;
    m_wholeBodyControlBoardInterfaces.axis = nullptr;
    m_wholeBodyControlBoardInterfaces.ctrlMode = nullptr;
    m_controlActive = false;
    m_sensorsReadingsAvailable = false;
}

void BridgeIHMCORS::onDesiredMessageReceived(const it::iit::yarp::RobotDesireds& receivedRobotDesired)
{
    if (receivedRobotDesired.jointDesireds().size() != m_jointTypes.size())
    {
        yError() << "BridgeIHMCORS: Wrong number of joints in it::iit::yarp::RobotDesireds received message";
        return;
    }

    // If no sensors was available, to not run the "IHMC"-joint control loop
    if (!m_sensorsReadingsAvailable)
    {
        return;
    }

    // Handle control mode

    // Reading the control mode is fast
    bool needToSetControlModeToTorque = false;
    m_wholeBodyControlBoardInterfaces.ctrlMode->getControlModes(m_measuredControlModes.data());
    for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
    {
        if (m_measuredControlModes[jnt] != VOCAB_CM_TORQUE)
        {
            needToSetControlModeToTorque = true;
        }
    }

    // todo(traversaro): check if this blocking call is problematic
    if (needToSetControlModeToTorque)
    {
        m_wholeBodyControlBoardInterfaces.ctrlMode->setControlModes(m_controlModeTorque.data());
    }

    // Compute desired torques using the IHMC control law
    m_sensorReadingsMutex.lock();
    for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
    {
        const it::iit::yarp::JointDesired& jd = receivedRobotDesired.jointDesireds()[jnt];
        m_desiredTorques[jnt] = jd.tau() +
                                jd.kp()*( jd.qDesired()  - deg2rad(m_jointPositionsFromYARPInDeg[jnt]) ) +
                                jd.kd()*( jd.qdDesired() - deg2rad(m_jointVelocitiesFromYARPInDegPerSec[jnt]) );
    }
    m_sensorReadingsMutex.unlock();

    // Send desired joint torques
    m_wholeBodyControlBoardInterfaces.trqs->setRefTorques(m_desiredTorques.data());

    // Set the state of the control to active and save the last instant in which the feedback message have been received
    m_controlActive = true;
    m_lastTimeOfReceivedDesiredMessage = yarp::os::Time::now();
}

}
}
