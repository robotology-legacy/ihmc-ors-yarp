// Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
// Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#include "BridgeIHMCORS.h"

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

#include <yarp/math/Math.h>

#include <cassert>
#include <climits>
#include <cmath>

namespace yarp
{
namespace dev
{

//MARK: - Thread implementation

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

//MARK: - Bridge - PortReader implementation

    class BridgeIHMCORS::PortReader
    : public yarp::os::TypedReaderCallback<yarp::sig::Vector>
    {
    private:
        std::mutex& m_lock;
        std::vector<double>& m_outputBuffer;
        yarp::os::BufferedPort<yarp::sig::Vector> m_port;
        std::string m_destinationPort;

    public:

        yarp::os::BufferedPort<yarp::sig::Vector>& port() { return m_port; }
        std::string& destinationPort() { return m_destinationPort; }

        PortReader(std::mutex& mutex, std::vector<double>& outputBuffer)
        : m_lock(mutex)
        , m_outputBuffer(outputBuffer)
        {
            m_port.useCallback(*this);
        }

        virtual void onRead(yarp::sig::Vector& newData)
        {
            std::lock_guard<std::mutex> guard(m_lock);
            m_outputBuffer.resize(newData.size()); //hopefully a no-op after the first time
            std::memcpy(m_outputBuffer.data(), newData.data(), sizeof(double) * m_outputBuffer.size());
        }

    };


//MARK: - Bridge implementation

BridgeIHMCORS::BridgeIHMCORS(): os::RateThread(5), m_feedbackSocket(nullptr)
{
    resetInterfaces();
}

BridgeIHMCORS::~BridgeIHMCORS()
{

}


bool BridgeIHMCORS::configurePortBasedMeasurements(const std::string& parameter,
                                                   const yarp::os::Searchable& config,
                                                   std::vector<std::vector<double>> &outputBuffer,
                                                   std::vector<std::shared_ptr<PortReader>>& configuredPorts)
{
    using namespace yarp::os;

    Bottle& group = config.findGroup(parameter);
    if (group.isNull()) {
        yWarning("Group %s not found in configuration", parameter.c_str());
        return true;
    }

    // well formed group has two elements:
    // - 1: group key
    // - 2: Bottle containing the actual value

    if (group.size() < 2
        || !group.get(1).isList()
        || !group.get(1).asList()) {
        yError("Malformed group for %s: %s", parameter.c_str(), group.toString().c_str());
    }

    // get the second element
    Bottle* content = group.get(1).asList();

    // For each element in the group, search for the port name
    // and configure the port
    configuredPorts.resize(content->size());
    outputBuffer.resize(content->size());
    for (int i = 0; i < content->size(); ++i) {
        // get the ID
        std::string elementID = content->get(i).asString();
        // find the corresponding element
        std::string portName = config.find(elementID).asString();
        if (portName.empty()) return false;

        configuredPorts[i] = std::make_shared<PortReader>(m_sensorReadingsMutex, outputBuffer[i]);
        if (!configuredPorts[i]) {
            yError("Failed to create Port");
            return false;
        }

        // Create unnamed port
        if (!configuredPorts[i]->port().open("...")) {
            yError("Failed to open (unnamed) port %s", configuredPorts[i]->port().getName().c_str());
            return false;
        }

        // Save connection destination for later (we connect in attach)
        configuredPorts[i]->destinationPort() = portName;
    }

    return true;
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

    // Configuring reading of IMUs and FTs
    if (!configurePortBasedMeasurements("imus", config, m_imusReadings, m_imuPorts)) {
        yError("Failed to configure IMUs readings");
        return false;
    }
    if (!configurePortBasedMeasurements("fts", config, m_ftsReadings, m_ftPorts)) {
        yError("Failed to configure FTs readings");
        return false;
    }

    // resize constant buffers
    m_orientationBuffer.resize(4, 4);
    m_orientationBuffer.zero();
    m_rpyBuffer.resize(3);
    m_rpyBuffer.zero();
    m_orientationBuffer = yarp::math::rpy2dcm(m_rpyBuffer);


    yInfo("Bridge configured successfully");

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
        ok = ok && this->start();

        // Thread receiving the desired values
        ok = ok && m_receivingThread.start();
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
            ok = ok && p[devIdx]->poly->view(m_wholeBodyControlBoardInterfaces.posdir);

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
            // Resize buffers for FTs and IMUs
            for (auto& buffer : m_imusReadings) {
                buffer.resize(12);
            }
            for (auto& buffer : m_ftsReadings) {
                buffer.resize(6);
            }

            m_measuredControlModes.resize(nj);
            m_controlModeTorque.resize(nj, VOCAB_CM_TORQUE);
            m_controlModePosition.resize(nj, VOCAB_CM_POSITION);

            // Resize buffers
            m_robotFeedback.jointStates().resize(nj);
            m_robotFeedback.imuStates().resize(m_imusReadings.size());
            m_robotFeedback.forceSensors().resize(m_ftPorts.size());

            // Resize FastCDR feedback buffer to the size of m_robotFeedback
            m_feedbackBuffer.resize(m_robotFeedback.getCdrSerializedSize(m_robotFeedback));

            // Resize the internal buffers of the receiving thread
            m_receivingThread.resize(nj);

            // Resize control mode
            m_jointORSControlModeMutex.lock();
            m_jointORSControlMode.resize(nj);
            for(int jnt=0; jnt < nj; jnt++)
            {
                m_jointORSControlMode[jnt] = it::iit::yarp::NOT_ENABLED;
            }
            m_jointORSControlModeMutex.unlock();

            // Reserve this buffers (they are actually filled only if necessary)
            m_desiredControlModesJnt.reserve(nj);
            m_desiredControlModes.reserve(nj);
            m_desTorquesJnt.reserve(nj);
            m_desTorques.reserve(nj);
            m_desPosJnt.reserve(nj);
            m_desPos.reserve(nj);
            m_desiredControlModesJntForTimeout.reserve(nj);
            m_desiredControlModesForTimeout.reserve(nj);


            // Connect the ports for IMUs and FTs
            if (ok)
            {
                for (auto& port : m_imuPorts) {
                    // Connect it to the output port
                    ok = yarp::os::Network::connect(port->destinationPort(), port->port().getName());
                    if (!ok) {
                        yError("Failed to connect %s to %s", port->destinationPort().c_str(), port->port().getName().c_str());
                        break;
                    }
                }

                for (auto& port : m_ftPorts) {
                    // Connect it to the output port
                    ok = yarp::os::Network::connect(port->destinationPort(), port->port().getName());
                    if (!ok) {
                        yError("Failed to connect %s to %s", port->destinationPort().c_str(), port->port().getName().c_str());
                        break;
                    }
                }
            }


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

        m_sensorReadingsMutex.lock();
        // Copying last available IMU
        for (size_t i = 0; i < m_imusReadings.size(); ++i) {
            const std::vector<double>& currentSensor = m_imusReadings[i];
            it::iit::yarp::IMUState& outputData = m_robotFeedback.imuStates()[i];
            //convert from RPY to quaternion
            m_rpyBuffer(0) = currentSensor[0];
            m_rpyBuffer(1) = currentSensor[1];
            m_rpyBuffer(2) = currentSensor[2];
            yarp::math::Quaternion quaternion = quaternionFromRPY(m_rpyBuffer);
            outputData.qs(quaternion.w());
            outputData.qx(quaternion.x());
            outputData.qy(quaternion.y());
            outputData.qz(quaternion.z());
            outputData.xdd() = currentSensor[3];
            outputData.ydd() = currentSensor[4];
            outputData.zdd() = currentSensor[5];
            outputData.wx() = deg2rad(currentSensor[6]);
            outputData.wy() = deg2rad(currentSensor[7]);
            outputData.wz() = deg2rad(currentSensor[8]);
        }

        // Copying last available FTs
        for (size_t i = 0; i < m_ftsReadings.size(); ++i) {
            const std::vector<double>& currentSensor = m_ftsReadings[i];
            it::iit::yarp::ForceSensor& outputData = m_robotFeedback.forceSensors()[i];
            outputData.fx() = currentSensor[0];
            outputData.fy() = currentSensor[1];
            outputData.fz() = currentSensor[2];
            outputData.tauX() = currentSensor[3];
            outputData.tauY() = currentSensor[4];
            outputData.tauZ() = currentSensor[5];
        }
        m_sensorReadingsMutex.unlock();

        // Serialize m_robotFeedback and send message using udp
        Cdr robotFeedbackSer(m_feedbackBuffer);
        m_robotFeedback.serialize(robotFeedbackSer);
        m_feedbackSocket->send_to(asio::buffer(m_feedbackBuffer.getBuffer(), m_feedbackBuffer.getBufferSize()), m_feedbackEndpoint);
    }
    else
    {
        yWarning() << "BridgeIHMCORS: some sensor were not read correctly";
    }

    // Handle timeout of desired messages, if there are any enabled joints
    bool areSomeJointsEnabled = false;
    m_desiredControlModesJntForTimeout.resize(0);
    m_desiredControlModesForTimeout.resize(0);
    m_jointORSControlModeMutex.lock();
    for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
    {
        if (m_jointORSControlMode[jnt] != it::iit::yarp::NOT_ENABLED)
        {
            areSomeJointsEnabled = true;
            m_desiredControlModesJntForTimeout.push_back(jnt);
            m_desiredControlModesForTimeout.push_back(VOCAB_CM_POSITION);
        }
    }
    m_jointORSControlModeMutex.unlock();

    if (areSomeJointsEnabled)
    {
        double timeSinceLastFeedback = yarp::os::Time::now()-m_lastTimeOfReceivedDesiredMessage;

        if (timeSinceLastFeedback > m_desiredTimeoutInSeconds)
        {
            m_jointORSControlModeMutex.lock();
            for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
            {
                if (m_jointORSControlMode[jnt] != it::iit::yarp::NOT_ENABLED)
                {
                    m_desiredControlModesJntForTimeout.push_back(jnt);
                    m_desiredControlModesForTimeout.push_back(VOCAB_CM_POSITION);
                    m_jointORSControlMode[jnt] = it::iit::yarp::NOT_ENABLED;
                }
            }
            m_jointORSControlModeMutex.unlock();

            // Warning: this is a blocking call, check if it is problematic
            m_wholeBodyControlBoardInterfaces.ctrlMode->setControlModes(m_desiredControlModesJntForTimeout.size(),
                                                                        m_desiredControlModesJntForTimeout.data(),
                                                                        m_desiredControlModesForTimeout.data());
            yInfo("BridgeIHMCORS: %lf seconds passed without receiving a message from the IHMC-ORS controller, switching controlled joints back to position mode.", timeSinceLastFeedback);
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

    // Disconnect the ports for IMUs and FTs
    for (auto& port : m_imuPorts) {
        // Connect it to the output port
        if (!yarp::os::Network::disconnect(port->destinationPort(), port->port().getName())) {
            yError("Failed to disconnect %s to %s", port->destinationPort().c_str(), port->port().getName().c_str());
            break;
        }
    }

    for (auto& port : m_ftPorts) {
        // Connect it to the output port
        if (!yarp::os::Network::disconnect(port->destinationPort(), port->port().getName())) {
            yError("Failed to disconnect %s to %s", port->destinationPort().c_str(), port->port().getName().c_str());
            break;
        }
    }


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

    for (auto& port : m_imuPorts) {
        port->port().interrupt();
        port->port().close();
    }

    for (auto& port : m_ftPorts) {
        port->port().interrupt();
        port->port().close();
    }

    return true;
}

void BridgeIHMCORS::resetInterfaces()
{
    m_wholeBodyControlBoardInterfaces.encs = nullptr;
    m_wholeBodyControlBoardInterfaces.trqs = nullptr;
    m_wholeBodyControlBoardInterfaces.axis = nullptr;
    m_wholeBodyControlBoardInterfaces.ctrlMode = nullptr;
    m_wholeBodyControlBoardInterfaces.posdir = nullptr;
    m_sensorsReadingsAvailable = false;
}

yarp::math::Quaternion BridgeIHMCORS::quaternionFromRPY(const yarp::sig::Vector& rpyBuffer) const
{
    m_orientationBuffer = yarp::math::rpy2dcm(rpyBuffer);
    yarp::math::Quaternion q;
    q.fromRotationMatrix(m_orientationBuffer);
    return q;
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

    // Reading the control mode is fast, so there is no problem in reading them
    bool needToSetControlModeToTorque = false;
    m_wholeBodyControlBoardInterfaces.ctrlMode->getControlModes(m_measuredControlModes.data());

    // We check the actual control mode of each joint and save the one that need to be changed
    // Note: a single call to setControlModes can be quite faster than several calls to setControlMode
    m_desiredControlModesJnt.resize(0);
    m_desiredControlModes.resize(0);
    m_jointORSControlModeMutex.lock();
    for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
    {
        // For each joint, we have three relevant variables
        // * The YARP controlmode, contained in m_measuredControlModes[jnt]
        // * The current ORS controlmode, contained in m_jointORSControlMode[jnt]
        // * The newly requested ORS controlmode, contained in receivedRobotDesired.jointDesireds()[jnt].controlMode
        // Depending on this variables, a new command is send to YARP or not
        const it::iit::yarp::JointDesired& jd = receivedRobotDesired.jointDesireds()[jnt];

        if ((m_measuredControlModes[jnt] != VOCAB_CM_TORQUE) && (jd.controlMode() == it::iit::yarp::TORQUE_CONTROL))
        {
            m_desiredControlModesJnt.push_back(static_cast<int>(jnt));
            m_desiredControlModes.push_back(VOCAB_CM_TORQUE);
        }

        if ((m_measuredControlModes[jnt] != VOCAB_CM_POSITION_DIRECT) && (jd.controlMode() == it::iit::yarp::POSITION_CONTROL))
        {
            m_desiredControlModesJnt.push_back(static_cast<int>(jnt));
            m_desiredControlModes.push_back(VOCAB_CM_POSITION_DIRECT);
        }

        // We only set a NOT_ENABLED joint in VOCAB_CM_POSITION mode only if before it was not NOT_ENABLED
        if ((m_measuredControlModes[jnt] != VOCAB_CM_POSITION) &&
            (jd.controlMode() == it::iit::yarp::NOT_ENABLED) &&
            (m_jointORSControlMode[jnt] != it::iit::yarp::NOT_ENABLED))
        {
            m_desiredControlModesJnt.push_back(static_cast<int>(jnt));
            m_desiredControlModes.push_back(VOCAB_CM_POSITION);
        }

        m_jointORSControlMode[jnt] = jd.controlMode();
    }
    m_jointORSControlModeMutex.unlock();

    // todo(traversaro): check if this blocking call is problematic
    if (m_desiredControlModesJnt.size() > 0)
    {
        m_wholeBodyControlBoardInterfaces.ctrlMode->setControlModes(m_desiredControlModesJnt.size(),
                                                                    m_desiredControlModesJnt.data(),
                                                                    m_desiredControlModes.data());
    }

    // Compute desired torques using the IHMC control law
    // todo(traversaro) this should be moved to a separate thread so it can run at an higher rate w.r.t.
    //                  when the desired message is received from the IHMC-ORS controller
    m_desTorquesJnt.resize(0);
    m_desTorques.resize(0);
    m_desPosJnt.resize(0);
    m_desPos.resize(0);
    m_sensorReadingsMutex.lock();
    m_jointORSControlModeMutex.lock();
    for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
    {
        const it::iit::yarp::JointDesired &jd = receivedRobotDesired.jointDesireds()[jnt];

        if (m_jointORSControlMode[jnt] == it::iit::yarp::TORQUE_CONTROL)
        {
            double desTorque = jd.tau() +
                               jd.kp() * (jd.qDesired() - deg2rad(m_jointPositionsFromYARPInDeg[jnt])) +
                               jd.kd() * (jd.qdDesired() - deg2rad(m_jointVelocitiesFromYARPInDegPerSec[jnt]));
            m_desTorquesJnt.push_back(static_cast<int>(jnt));
            m_desTorques.push_back(desTorque);
        }

        if (m_jointORSControlMode[jnt] == it::iit::yarp::POSITION_CONTROL)
        {
            double desPosInDeg  = rad2deg(jd.qDesired());
            m_desPosJnt.push_back(static_cast<int>(jnt));
            m_desPos.push_back(desPosInDeg);
        }
    }
    m_jointORSControlModeMutex.unlock();
    m_sensorReadingsMutex.unlock();

    // Send desired joint torques (if any)
    if (m_desTorquesJnt.size() > 0)
    {
        m_wholeBodyControlBoardInterfaces.trqs->setRefTorques(m_desTorquesJnt.size(),
                                                              m_desTorquesJnt.data(),
                                                              m_desTorques.data());
    }

    // Send desired joint positions (if any)
    if (m_desPosJnt.size() > 0)
    {
        m_wholeBodyControlBoardInterfaces.posdir->setPositions(m_desPosJnt.size(),
                                                               m_desPosJnt.data(),
                                                               m_desPos.data());
    }

    // Save the last instant in which the feedback message have been received
    m_lastTimeOfReceivedDesiredMessage = yarp::os::Time::now();
}

}
}
