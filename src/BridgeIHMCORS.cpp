// Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
// Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#include "BridgeIHMCORS.h"

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <cassert>
#include <climits>
#include <cmath>

namespace yarp
{
namespace dev
{

BridgeIHMCORS::BridgeIHMCORS(): os::RateThread(5)
{
    resetInterfaces();
}

BridgeIHMCORS::~BridgeIHMCORS()
{

}

bool BridgeIHMCORS::open(yarp::os::Searchable& config)
{
    double periodInSeconds=config.check("period",os::Value(0.005)).asDouble();
    int period_ms=int(1000.0*periodInSeconds);
    this->setRate(period_ms);
    return true;
}

bool BridgeIHMCORS::attachAll(const PolyDriverList& p)
{
    yarp::os::LockGuard guard(m_deviceMutex);

    bool ok = true;
    ok = ok && attachWholeBodyControlBoard(p);

    if (ok)
    {
        this->start();
    }

    // TODO(traversaro) : at this point, everything is ready, so we should also attach in someway a callback
    //                    on a received desired message to onDesiredMessageReceived

    m_correctlyConfigured = ok;

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

            m_jointPositionsFromYARP.resize(nj);
            m_jointVelocitiesFromYARP.resize(nj);
	    m_jointTorquesFromYARP.resize(nj);

            // Resize buffers
            m_robotFeedback.jointStates().resize(nj);
            m_robotFeedback.forceSensors().resize(0);
            m_robotFeedback.imuStates().resize(0);

            m_desiredTorques.resize(nj);

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
    yarp::os::LockGuard guard(m_deviceMutex);

    if( m_correctlyConfigured )
    {
        bool sensorsReadCorrectly = true;
        sensorsReadCorrectly = sensorsReadCorrectly && m_wholeBodyControlBoardInterfaces.encs->getEncoders(m_jointPositionsFromYARP.data());
        sensorsReadCorrectly = sensorsReadCorrectly && m_wholeBodyControlBoardInterfaces.encs->getEncoderSpeeds(m_jointVelocitiesFromYARP.data());
	sensorsReadCorrectly = sensorsReadCorrectly && m_wholeBodyControlBoardInterfaces.trqs->getTorques(m_jointTorquesFromYARP.data());

        if (sensorsReadCorrectly)
        {
            for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
            {
                if (m_jointTypes[jnt] == VOCAB_JOINTTYPE_REVOLUTE)
                {
                    m_robotFeedback.jointStates()[jnt].q() = deg2rad(m_jointPositionsFromYARP[jnt]);
                    m_robotFeedback.jointStates()[jnt].qd() = deg2rad(m_jointVelocitiesFromYARP[jnt]);
                }
                else
                {
                    m_robotFeedback.jointStates()[jnt].q() = m_jointPositionsFromYARP[jnt];
                    m_robotFeedback.jointStates()[jnt].qd() = m_jointVelocitiesFromYARP[jnt];
                }
                m_robotFeedback.jointStates()[jnt].tau() = m_jointTorquesFromYARP[jnt];

                // std::cerr << "m_robotFeedback message updated " << std::endl;
            }

        }
        else
        {
            yWarning() << "bridgeIHMCORS warning : some sensor were not readed correctly";
        }
    }
}

bool BridgeIHMCORS::detachAll()
{
    yarp::os::LockGuard guard(m_deviceMutex);

    if (isRunning())
    {
        stop();
    }

    // We should disconnect the callback on the new message now

    resetInterfaces();

    return true;
}

bool BridgeIHMCORS::close()
{
}

void BridgeIHMCORS::resetInterfaces()
{
    m_correctlyConfigured = false;
    m_wholeBodyControlBoardInterfaces.encs = nullptr;
    m_wholeBodyControlBoardInterfaces.trqs = nullptr;
    m_wholeBodyControlBoardInterfaces.axis = nullptr;
}

void BridgeIHMCORS::onDesiredMessageReceived(const it::iit::yarp::RobotDesireds& receivedRobotDesired)
{
    if (receivedRobotDesired.jointDesireds().size() != m_jointTypes.size())
    {
        yError() << "bridgeIHMCORS: Wrong number of joints in it::iit::yarp::RobotDesireds received message";
        return;
    }

    // TODO(traversaro): handle translation between control modes and enabled/disabled flags
    for (size_t jnt=0; jnt < m_jointTypes.size(); jnt++)
    {
        m_desiredTorques[jnt] = receivedRobotDesired.jointDesireds()[jnt].tau();
    }

    // Send desired joint torques
    m_wholeBodyControlBoardInterfaces.trqs->setRefTorques(m_desiredTorques.data());
}





}
}
