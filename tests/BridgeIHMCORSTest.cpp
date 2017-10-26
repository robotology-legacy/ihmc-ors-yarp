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

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/**********************************************************************/
class BridgeIHMCORSTest : public yarp::rtf::TestCase
{

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
    }

    /******************************************************************/
    virtual void tearDown()
    {
    }

    /******************************************************************/
    virtual void run()
    {
        RTF_TEST_REPORT("Computing walking trajectory");
    }
};

PREPARE_PLUGIN(BridgeIHMCORSTest)
