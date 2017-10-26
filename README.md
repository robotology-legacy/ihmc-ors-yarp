
ihmc-ors-yarp
=============

**Warning: this library is still in active development, and is not supposed to work at this stage**

The `ihmc-ors-yarp` project provides a bridge to interface YARP-powered robots with the [IHMC-ORS](https://github.com/ihmcrobotics/ihmc-open-robotics-software) software.
In particular `ihmc-ors-yarp` provides the `bridge_ihmc_ors` YARP devices, that can be launched through the `yarprobotinterface` to expose YARP controlboards to a controller
implemented using  [IHMC-ORS](https://github.com/ihmcrobotics/ihmc-open-robotics-software).

## Dependencies
ihmc-ors-yarp library depends on
 - [YARP](http://www.yarp.it/)
 - [Asio](https://think-async.com/)
 - [fastcdr](https://github.com/eProsima/Fast-CDR)

Furthermore, `ihmc-ors-yarp` relies on idl messages generated using the `fastrtpsgen` of [fastrtps](https://github.com/eProsima/Fast-RTPS).
However, unless you need to regenerate the message, you will not need `fastrtps` to build `ihmc-ors-yarp`.

## Installation
### Build
```shell
git clone https://github.com/robotology-playground/ihmc-ors-yarp
cd ihmc-ors-yarp
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<install_prefix> ..
make
[sudo] make install
```

### Configure your system
To make sure that the `bridge_ihmc_ors` YARP device is found, add the `<install_prefix>/share/yarp` to the `YARP_DATA_DIRS` env variable. To check that the device was correctly found by YARP, you should find this line if you run the command `yarpdev --list`: 
~~~
[INFO]Device "bridge_ihmc_ors", available on request (found in <install_prefix>/lib/yarp/bridge_ihmc_ors.so library).
~~~

## Usage

### iCub

#### Simulated
We provide a few configuration files to run the device on the iCub Gazebo simulation. in `robots/icubGazeboSim`:
~~~
yarprobotinterface --config launch-bridgeihmcors-sim.xml
~~~
As the `gazebo_yarp_plugins` do no support launching a YARP device inside Gazebo itself, this configuration file connects
the `bridge_ihmc_ors` to the controlboards through a group of `remote_controlboard` devices. Clearly this is not ideal, but
it is sufficient to testing. The ideal use of this plugin is to embed it in the `yarprobotinterface` that is running the
controlboard devices exposing the functionalities of the robot.

#### Real Robot
To run the device on the real robot, it should be sufficient to add this line to your `yarprobotinterface` configuration file:
~~~
    <!-- Remapper device that selects a given set of joints -->
    <device name="ihmc_remapped_controlboard" type="controlboardremapper">
         <param name="axesNames">(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,l_ankle_pitch,l_ankle_roll,r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)</param>

         <action phase="startup" level="10" type="attach">
            <paramlist name="networks">
                <!-- motorcontrol -->
                <elem name="left_leg">left_leg_mc</elem>
                <elem name="right_leg">right_leg_mc</elem>
                <elem name="left_arm">left_arm_mc</elem>
                <elem name="right_arm">right_arm_mc</elem>
                <elem name="torso">torso_mc</elem>
            </paramlist>
         </action>
    </device>

    <!-- actual bridgeIHMCORS device -->
    <device name="ihmc_bridge" type="bridge_ihmc_ors">
         <!-- Period in seconds at which the torque are sent to the robot 
              and at which the feedback message is sent to the IHMC-ORS controller -->
         <param name="period">0.005</param>
         <!-- IP address (of the machine running IHMC-ORS controller) to which the feedback datagram is sent --> 
         <param name="feedback-address">localhost</param>
         <!-- IP address (of the machine running the yarprobotinterface) from which the desired datagram is received --> 
         <param name="desired-address">localhost</param>
         <action phase="startup" level="20" type="attach">
            <paramlist name="networks">
                <!-- motorcontrol  -->
                <elem name="ihmc_remapped_controlboard">ihmc_remapped_controlboard</elem>
            </paramlist>
         </action>
    </device>
~~~
where the devices passed to the `ihmc_remapped_controlboard` are the actual controlboard devices of your robot (that may have different names) and
the `axesNames` parameter has the list of joints that you want to expose through the `bridge_ihmc_ors` device.

Regarding the `bridge_ihmc_ors` device, the relevant parameters are `period`, `feedback-address` and `desired-address`. 

For a complete documentation of these parameters, check the Doxygen documentation of the `BridgeIHMCORS` C++ class.


## Device behaviour 
By default, after the start the device send the RobotFeedback message as a UDP datagram to the `feedback-address` IP address 
on the port `feedback-port-number` (default value: `9970`). A separate threads listen for a UDP datagram containing the RobotDesireds message 
 on the `desired-address` IP address  on the port `desired-port-number` (default value: `9980`)
 
![state](https://user-images.githubusercontent.com/1857049/32063969-e05ff15c-ba78-11e7-9f20-5cb08d86f54d.png)

The bridge starts in the OFF state. As soon as it receives a RobotDesireds message, it switches to the ON state. While entering in the ON state, it switches all the YARP control modes of the joint handled by the bridge to `VOCAB_CM_TORQUE`.

While the bridge is in the ON state, it sends desired torques based on the control law specified in https://github.com/robotology-playground/ihmc-ors-yarp/issues/3, using data received from the RobotDesireds messages and the feedback received from the robot.  

Once more then 0.1 second pass without receiving any RobotDesireds message, the bridge switches back to the OFF state, and it switches back all the joint to the YARP control mode `VOCAB_CM_POSITION`.


## Regenerate the idl messages
To regenerate the idl messages, you need to have the `fastrtpsgen` tool on your PATH, that is part
of [`fastrtps`](https://github.com/eProsima/Fast-RTPS). If you compile `fastrtps` from source, remember
to enable the `BUILD_JAVA` CMake option to compile the `fastrtpsgen` tool.
Once you have `fastrtpsgen` in your PATH,  you can regenerate the autogenerated files in the `autogenerated` directory by executing the `idl-regenerate-cxx` target.
