# Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
# Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# For testing we require RTF and YARP compiled with RTF support
find_package(RTF 1.3.4 REQUIRED)
if(NOT TARGET YARP::YARP_rtf)
  message(FATAL_ERROR "BUILD_TESTING is enabled, but no RTF or YARP::YARP_rtf was detected, \
                       please recompile YARP with the YARP_COMPILE_RTF_ADDONS CMake option set to ON to compile the tests in this repo.")
endif()

rtf_add_plugin(BridgeIHMCORSTest SOURCES BridgeIHMCORSTest.cpp)

target_include_directories(BridgeIHMCORSTest PRIVATE ${Asio_INCLUDE_DIRS})
target_link_libraries(BridgeIHMCORSTest PRIVATE RTF::RTF RTF::RTF_dll YARP::YARP_OS YARP::YARP_dev YARP::YARP_rtf fastcdr idl_messages)

# Configure the testing fixture
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/fixture-sim-and-bridge.xml.in
               ${CMAKE_CURRENT_BINARY_DIR}/fixture-sim-and-bridge.xml @ONLY)

# Add the test
rtf_add_suite(NAME BridgeIHMCORSTestSuite
              FIXTURE yarpserver
              FIXTURE yarpmanager --fixture ${CMAKE_CURRENT_BINARY_DIR}/fixture-sim-and-bridge.xml
              TEST BridgeIHMCORSTest TYPE dll
              VERBOSE)
