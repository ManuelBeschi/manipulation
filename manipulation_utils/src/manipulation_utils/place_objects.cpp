
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <manipulation_utils/place_objects.h>

#include <object_loader_msgs/detachObject.h>

namespace manipulation
{
  PlaceObjects::PlaceObjects( const ros::NodeHandle& nh, 
                              const ros::NodeHandle& pnh):
                              m_nh(nh),
                              m_pnh(pnh),
                              SkillBase(nh,pnh)
  {

  }

  bool PlaceObjects::init()
  {
    if (!SkillBase::init())
    {
      m_init = false;
      return m_init;
    }
    
    m_detach_object_srv = m_nh.serviceClient<object_loader_msgs::detachObject>("detach_object_to_link");
    m_reset_srv = m_nh.advertiseService("outbound/reset",&PlaceObjects::resetCb,this);
     
    for (const std::string& group_name: m_group_names)
    {
      m_as.at(group_name)->start();
    }

    m_init = true;
    return m_init;

  }

    bool SkillBase::addObjectsCb( manipulation_msgs::AddObjects::Request& req,
                                manipulation_msgs::AddObjects::Response& res)
  {
    for (const manipulation_msgs::Objects& object: req.add_objects )
    {
      if(!m_loc_man.addLocationsFromMsg(location))
        return false;
    }

    // To be done handle the results

    return true;
  }

  bool SkillBase::addBoxesCb( manipulation_msgs::AddBoxes::Request& req,
                              manipulation_msgs::AddBoxes::Response& res)
  {
    for (const manipulation_msgs::Objects& box: req.add_boxes )
    {
      if(!m_loc_man.addLocationsFromMsg(box.location))
        return false;
    }
 
    // To be done handle the results

    return true;
  }

  bool addLocationsCb(manipulation_msgs::AddLocations::Request& req,
                      manipulation_msgs::AddLocations::Response& res)
  { 
    for (const manipulation_msgs::Location& location: req.locations )
    {
      if(!m_loc_man.addLocationsFromMsg(location))
        return false;
    }

    // To be done handle the results

    return true;
  } 

  bool listObjectsCb( manipulation_msgs::ListOfObjects::Request& req,
                      manipulation_msgs::ListOfObjects::Response& res)
  {
      
  }

  bool resetBoxesCb(std_srvs::SetBool::Request& req, 
                    std_srvs::SetBool::Response& res) 
  {
    // to be evaluated
  }
                    
  bool removeLocationsCb( manipulation_msgs::RemoveLocations::Request& req,
                          manipulation_msgs::RemoveLocations::Response& res)
  {
    if (!m_loc_man.addLocationsFromMsg(req.location_names))
      return false;

    return true;
  }

}