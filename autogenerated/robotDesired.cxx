// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file robotDesired.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "robotDesired.h"

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>


it::iit::yarp::JointDesired::JointDesired()
{
    m_controlMode = it::iit::yarp::NOT_ENABLED;
    m_tau = 0.0;
    m_kp = 0.0;
    m_kd = 0.0;
    m_qDesired = 0.0;
    m_qdDesired = 0.0;
}

it::iit::yarp::JointDesired::~JointDesired()
{
}

it::iit::yarp::JointDesired::JointDesired(const JointDesired &x)
{
    m_controlMode = x.m_controlMode;
    m_tau = x.m_tau;
    m_kp = x.m_kp;
    m_kd = x.m_kd;
    m_qDesired = x.m_qDesired;
    m_qdDesired = x.m_qdDesired;
}

it::iit::yarp::JointDesired::JointDesired(JointDesired &&x)
{
    m_controlMode = x.m_controlMode;
    m_tau = x.m_tau;
    m_kp = x.m_kp;
    m_kd = x.m_kd;
    m_qDesired = x.m_qDesired;
    m_qdDesired = x.m_qdDesired;
}

it::iit::yarp::JointDesired& it::iit::yarp::JointDesired::operator=(const JointDesired &x)
{
    m_controlMode = x.m_controlMode;
    m_tau = x.m_tau;
    m_kp = x.m_kp;
    m_kd = x.m_kd;
    m_qDesired = x.m_qDesired;
    m_qdDesired = x.m_qdDesired;
    
    return *this;
}

it::iit::yarp::JointDesired& it::iit::yarp::JointDesired::operator=(JointDesired &&x)
{
    m_controlMode = x.m_controlMode;
    m_tau = x.m_tau;
    m_kp = x.m_kp;
    m_kd = x.m_kd;
    m_qDesired = x.m_qDesired;
    m_qdDesired = x.m_qdDesired;
    
    return *this;
}

size_t it::iit::yarp::JointDesired::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    return current_alignment - initial_alignment;
}

size_t it::iit::yarp::JointDesired::getCdrSerializedSize(const it::iit::yarp::JointDesired& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    return current_alignment - initial_alignment;
}

void it::iit::yarp::JointDesired::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << (uint32_t)m_controlMode;
    scdr << m_tau;
    scdr << m_kp;
    scdr << m_kd;
    scdr << m_qDesired;
    scdr << m_qdDesired;
}

void it::iit::yarp::JointDesired::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> (uint32_t&)m_controlMode;
    dcdr >> m_tau;
    dcdr >> m_kp;
    dcdr >> m_kd;
    dcdr >> m_qDesired;
    dcdr >> m_qdDesired;
}

size_t it::iit::yarp::JointDesired::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            







    return current_align;
}

bool it::iit::yarp::JointDesired::isKeyDefined()
{
    return false;
}

void it::iit::yarp::JointDesired::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	 
	 
	 
	 
	 
	 
}
it::iit::yarp::RobotDesireds::RobotDesireds()
{
}

it::iit::yarp::RobotDesireds::~RobotDesireds()
{
}

it::iit::yarp::RobotDesireds::RobotDesireds(const RobotDesireds &x)
{
    m_jointDesireds = x.m_jointDesireds;
}

it::iit::yarp::RobotDesireds::RobotDesireds(RobotDesireds &&x)
{
    m_jointDesireds = std::move(x.m_jointDesireds);
}

it::iit::yarp::RobotDesireds& it::iit::yarp::RobotDesireds::operator=(const RobotDesireds &x)
{
    m_jointDesireds = x.m_jointDesireds;
    
    return *this;
}

it::iit::yarp::RobotDesireds& it::iit::yarp::RobotDesireds::operator=(RobotDesireds &&x)
{
    m_jointDesireds = std::move(x.m_jointDesireds);
    
    return *this;
}

size_t it::iit::yarp::RobotDesireds::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    for(size_t a = 0; a < 100; ++a)
    {
        current_alignment += it::iit::yarp::JointDesired::getMaxCdrSerializedSize(current_alignment);}

    return current_alignment - initial_alignment;
}

size_t it::iit::yarp::RobotDesireds::getCdrSerializedSize(const it::iit::yarp::RobotDesireds& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    for(size_t a = 0; a < data.jointDesireds().size(); ++a)
    {
        current_alignment += it::iit::yarp::JointDesired::getCdrSerializedSize(data.jointDesireds().at(a), current_alignment);}

    return current_alignment - initial_alignment;
}

void it::iit::yarp::RobotDesireds::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_jointDesireds;
}

void it::iit::yarp::RobotDesireds::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_jointDesireds;
}

size_t it::iit::yarp::RobotDesireds::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            

    return current_align;
}

bool it::iit::yarp::RobotDesireds::isKeyDefined()
{
    return false;
}

void it::iit::yarp::RobotDesireds::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	 
}


