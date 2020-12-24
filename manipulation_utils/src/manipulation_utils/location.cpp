/*
Copyright (c) 2020, Manuel Beschi 
CARI Joint Research Lab
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
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

#include <manipulation_utils/location.h>



namespace pickplace {

Location::Location(const std::string& name,
         const Eigen::Affine3d& T_w_location,
         const Eigen::Affine3d& T_w_approach,
         const Eigen::Affine3d& T_w_leave):
  m_name(name),
  m_T_w_location(T_w_location),
  m_T_w_approach(T_w_approach),
  m_T_w_return(T_w_leave)
{

}


Location::Location(const manipulation_msgs::Location &msg)
{
  tf::poseMsgToEigen(msg.pose,m_T_w_location);
  Eigen::Affine3d T_location_approach;
  tf::poseMsgToEigen(msg.approach_relative_pose,T_location_approach);
  m_T_w_approach=m_T_w_location*T_location_approach;

  Eigen::Affine3d T_location_return;
  tf::poseMsgToEigen(msg.approach_relative_pose,T_location_return);
  m_T_w_return=m_T_w_location*T_location_return;
  m_name=msg.id;
}

bool Location::canBePickedBy(const std::string &group_name)
{
  return ( m_location_configurations.find(group_name) != m_location_configurations.end() );
}

void Location::addLocationIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_location_configurations.find(group_name) == m_location_configurations.end() )
    m_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_location_configurations.at(group_name)=solutions;
}
void Location::addApproachIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_approach_location_configurations.find(group_name) == m_approach_location_configurations.end() )
    m_approach_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_approach_location_configurations.at(group_name)=solutions;
}
void Location::addReturnIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_return_location_configurations.find(group_name) == m_return_location_configurations.end() )
    m_return_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_return_location_configurations.at(group_name)=solutions;
}



}  // end namespace pickplace

