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
 */
class bridgeIHMCORS :  public yarp::dev::DeviceDriver,
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
    } m_wholeBodyControlBoardInterfaces;

    bool m_correctlyConfigured;

    // Helper methods to deal with C++ interfaces
    bool attachWholeBodyControlBoard(const PolyDriverList& p);
    void resetInterfaces();

    // Local buffers for readings sensors
    std::vector<yarp::dev::JointTypeEnum> m_jointTypes;
    std::vector<double> m_jointPositionsFromYARP;
    std::vector<double> m_jointVelocitiesFromYARP;
    
    // FastRTPS robot feedback message 
    it::iit::yarp::robotFeedback m_robotFeedback;

    // Function to call when a new desired message has been received
    // TODO(traversaro): make sure that this function is called
    void onDesiredMessageReceived(const it::iit::yarp::RobotDesireds& receivedRobotDesired);

    // Local buffers for sending desired values
    std::vector<double> m_desiredTorques;
    
public:
    // CONSTRUCTOR
    bridgeIHMCORS();
    ~bridgeIHMCORS();

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
