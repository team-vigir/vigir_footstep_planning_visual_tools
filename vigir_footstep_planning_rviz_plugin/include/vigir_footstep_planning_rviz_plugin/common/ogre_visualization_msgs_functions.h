//=================================================================================================
// Copyright (c) 2018, Stephanie Ferreira, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#ifndef OGRE_VISUALIZATION_MSGS_FUNCTIONS_H
#define OGRE_VISUALIZATION_MSGS_FUNCTIONS_H

#include <geometry_msgs/Pose.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>


inline Ogre::Vector3 getOgreVector(const geometry_msgs::Point& position)
{
  return Ogre::Vector3(position.x, position.y, position.z);
}

inline Ogre::Quaternion getOgreQuaternion(const geometry_msgs::Quaternion& orientation)
{
  return Ogre::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
}

inline geometry_msgs::Point getPositionMsg(const Ogre::Vector3& position)
{
  geometry_msgs::Point p;
  p.x = position.x;
  p.y = position.y;
  p.z = position.z;
  return p;
}

inline geometry_msgs::Quaternion getQuaternionMsg(const Ogre::Quaternion& orientation)
{
  geometry_msgs::Quaternion o;
  o.w = orientation.w;
  o.x = orientation.x;
  o.y = orientation.y;
  o.z = orientation.z;
  return o;
}

inline void getOgrePose(const geometry_msgs::Pose& pose_in, Ogre::Vector3& position_out, Ogre::Quaternion& orientation_out)
{
  position_out = getOgreVector(pose_in.position);
  orientation_out = getOgreQuaternion(pose_in.orientation);
}

inline void getPoseMsg(geometry_msgs::Pose& pose_out, const Ogre::Vector3& position_in, const Ogre::Quaternion& orientation_in)
{
  pose_out.position = getPositionMsg(position_in);
  pose_out.orientation = getQuaternionMsg(orientation_in);
}
#endif
