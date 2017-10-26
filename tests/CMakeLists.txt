# Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
# Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# For testing we require RTF and YARP compiled with RTF support
find_package(RTF)
if(NOT RTF_FOUND OR NOT TARGET YARP::YARP_rtf OR NOT TARGET RTF::RTF)
  message(FATAL_ERROR "BUILD_TESTING is enabled, but no RTF or YARP::YARP_rtf was detected, \
                       please recompile YARP with the YARP_COMPILE_RTF_ADDONS CMake option set to ON to compile the tests in this repo.")
endif()

rtf_add_plugin(BridgeIHMCORSTest MODULE BridgeIHMCORSTest.cpp)

target_link_libraries(BridgeIHMCORSTest PRIVATE RTF::RTF RTF::RTF_dll YARP::YARP_rtf)

# Configure the testing fixture
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/fixture-sim-and-bridge.xml.in
               ${CMAKE_CURRENT_BINARY_DIR}/fixture-sim-and-bridge.xml @ONLY)

# Add the test
rtf_add_suite(NAME BridgeIHMCORSTestSuite
              FIXTURE yarpserver
              FIXTURE yarpmanager
              TEST BridgeIHMCORSTest
              VERBOSE)
